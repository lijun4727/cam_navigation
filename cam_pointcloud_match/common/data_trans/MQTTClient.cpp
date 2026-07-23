#include "MQTTClient.h"

extern "C"
{
#include "mongoose.h"
}

#include <chrono>
#include <condition_variable>
#include <memory>
#include <vector>
#include <deque>

namespace
{
	constexpr std::chrono::seconds kConnectTimeout(5);
	constexpr int kPollTimeoutMs = 100;
	constexpr uint64_t kReconnectDelayMs = 3000;
#define MQTT_KEEP_ALIVE_SEC 30
}

MQTTClient::MQTTClient() :
	m_mgr(new mg_mgr()),
	m_connection(nullptr),
	m_state(State::Stopped),
	m_stopPollThread(true),
	m_reconnectScheduled(false)
{
	mg_mgr_init(m_mgr);
}

MQTTClient::~MQTTClient()
{
	Disconnect();
	if (m_mgr)
	{
		mg_mgr_free(m_mgr);
		delete m_mgr;
		m_mgr = nullptr;
	}
}

bool MQTTClient::Connect(const std::string& url, const std::string& clienId)
{
	if (url.empty() || clienId.empty())
		return false;

	Disconnect();

	{
		std::lock_guard<std::mutex> lock(m_mutex);
		m_url = url;
		m_clientId = clienId;
		m_lastError.clear();
		m_state = State::Connecting;
		m_stopPollThread = false;
		m_reconnectScheduled = false;
	}

	StartPollThread();

	if (!OpenConnection(url, clienId))
	{
		{
			std::lock_guard<std::mutex> lock(m_mutex);
			m_lastError = "mg_mqtt_connect failed";
			m_state = State::Failed;
			m_stopPollThread = true;
		}
		StopPollThread();
		return false;
	}

	if (!WaitForConnectResult())
	{
		Disconnect();
		return false;
	}

	return true;
}

bool MQTTClient::PuplishTopic(const std::string& topic, const std::string& msg)
{
    std::lock_guard<std::mutex> lock(m_mutex);
	if (m_state == State::Connected && m_connection != nullptr)
	{
		PuplishTopicImp(topic, msg);
		return true;
	}

	m_pendingPublishes.emplace_back(topic, msg);

	return false;
}

bool MQTTClient::SubTopic(const std::string& topic, CallBack callback)
{
	if (topic.empty() || !callback)
		return false;

	std::lock_guard<std::mutex> lock(m_mutex);
	m_callbacks[topic] = std::move(callback);
	SubTopicImp(topic);
	return true;
}

void MQTTClient::Disconnect()
{
	mg_connection* connection = nullptr;
	{
		std::lock_guard<std::mutex> lock(m_mutex);
		connection = m_connection;
		m_connection = nullptr;
		m_state = State::Stopped;
		m_stopPollThread = true;
		m_reconnectScheduled = false;
	}

	if (connection != nullptr)
	{
		mg_mqtt_opts opts{};
		mg_mqtt_disconnect(connection, &opts);
		connection->is_closing = 1;
	}

	StopPollThread();

	if (m_mgr)
	{
		mg_mgr_free(m_mgr);
		mg_mgr_init(m_mgr);
	}
}

void MQTTClient::EventHandler(struct mg_connection* c, int ev, void* ev_data)
{
	auto* self = static_cast<MQTTClient*>(c->fn_data);
	if (self != nullptr)
		self->EventHandlerImp(c, ev, ev_data);
}

void MQTTClient::EventHandlerImp(struct mg_connection* c, int ev, void* ev_data)
{
	switch (ev)
	{
	case MG_EV_OPEN:
		if (m_url.empty())
			break;
		if (mg_url_is_ssl(m_url.c_str()))
		{
			mg_tls_opts tlsOpts{};
			tlsOpts.skip_verification = 1;
			mg_tls_init(c, &tlsOpts);
		}
		break;

	//case MG_EV_CONNECT:
	//{
	//	std::lock_guard<std::mutex> lock(m_mutex);

	//}
	//	break;

	case MG_EV_MQTT_OPEN:
	{
		m_state = State::Connected;
		m_connectCv.notify_all();

		std::lock_guard<std::mutex> lock(m_mutex);
		m_reconnectScheduled = false;
		for (const auto& item : m_callbacks)
		{
			SubTopicImp(item.first);
		}

		while (!m_pendingPublishes.empty() && m_connection != nullptr)
		{
			const auto& p = m_pendingPublishes.front();
			PuplishTopicImp(p.first, p.second);
			m_pendingPublishes.pop_front();
		}
	}
	break;

	case MG_EV_MQTT_MSG:
	{
		auto* msg = static_cast<mg_mqtt_message*>(ev_data);
		if (msg == nullptr)
			break;

		const std::string topic(msg->topic.buf, msg->topic.len);
		const std::string payload(msg->data.buf, msg->data.len);
		std::vector<CallBack> callbacks;
		{
			std::lock_guard<std::mutex> lock(m_mutex);
			for (const auto& item : m_callbacks)
			{
				if (TopicMatches(item.first, topic))
					callbacks.push_back(item.second);
			}
		}

		for (const auto& callback : callbacks)
		{
			std::unique_ptr<char[]> buffer(new char[payload.size() + 1]);
			memcpy(buffer.get(), payload.data(), payload.size());
			buffer[payload.size()] = '\0';
			callback(buffer.get(), static_cast<int>(payload.size()));
		}
	}
	break;

	case MG_EV_ERROR:
	{
		const char* errorText = ev_data != nullptr ? static_cast<const char*>(ev_data) : "unknown error";
		std::lock_guard<std::mutex> lock(m_mutex);
		m_lastError = errorText;
		if (m_state == State::Connecting)
			m_state = State::Failed;
	}
	m_connectCv.notify_all();

	break;

	case MG_EV_CLOSE:
	{
		State previousState = State::Stopped;
		bool shouldReconnect = false;
		{
			std::lock_guard<std::mutex> lock(m_mutex);
			previousState = m_state;
			if (c == m_connection)
				m_connection = nullptr;

			if (m_stopPollThread)
			{
				m_state = State::Stopped;
				m_reconnectScheduled = false;
			}
			else if (previousState == State::Connected || previousState == State::Reconnecting)
			{
				m_state = State::Failed;
				shouldReconnect = true;
			}
			else if (previousState == State::Connecting)
			{
				m_state = State::Failed;
			}
			else if (previousState != State::Failed)
			{
				m_state = State::Stopped;
			}
		}

		if (shouldReconnect)
		{
			ScheduleReconnect();
		}
	}
	m_connectCv.notify_all();
	break;

	case MG_EV_POLL:
	{
		// 2. MG_EV_POLL 是 Mongoose 事件循环每次轮询时触发的事件
		// 在这里判断是否需要发送心跳。通常由客户端库内部时间或您自己记录的上次发送时间来决定
		// 建议：每隔 (MQTT_KEEP_ALIVE_SEC / 2) 即 15 秒，调用一次 mg_mqtt_ping
		static uint64_t last_ping_time = 0;
		uint64_t now = mg_millis(); // 获取当前毫秒数

		// 如果距离上次发 Ping 超过了 15 秒（15000 毫秒）
		if (now - last_ping_time >= (MQTT_KEEP_ALIVE_SEC * 1000 / 2)) {
			mg_mqtt_ping(c);          // 发送心跳包
			last_ping_time = now;     // 更新时间
		}
	}
		break;

	default:
		break;
	}
}

void MQTTClient::StartPollThread()
{
	if (m_pollThread.joinable())
		return;

	m_pollThread = std::thread([this]()
		{
			while (true)
			{
				if (m_stopPollThread)
					break;
				mg_mgr_poll(m_mgr, kPollTimeoutMs);
			}
		});
}

void MQTTClient::StopPollThread()
{
	if (m_pollThread.joinable())
		m_pollThread.join();
}

bool MQTTClient::OpenConnection(const std::string& url, const std::string& clientId)
{
	mg_mqtt_opts opts{};
	opts.client_id = mg_str(clientId.c_str());
	opts.clean = false;
	opts.keepalive = MQTT_KEEP_ALIVE_SEC;
	opts.version = 4;

	mg_connection* connection = mg_mqtt_connect(m_mgr, url.c_str(), &opts, &MQTTClient::EventHandler, this);
	if (connection == nullptr)
		return false;

	std::lock_guard<std::mutex> lock(m_mutex);
	if (m_stopPollThread)
	{
		connection->is_closing = 1;
		return false;
	}

	m_connection = connection;
	return true;
}

void MQTTClient::ScheduleReconnect()
{
	bool shouldSchedule = false;
	{
		std::lock_guard<std::mutex> lock(m_mutex);
		if (!m_stopPollThread && !m_reconnectScheduled && !m_url.empty() && !m_clientId.empty())
		{
			m_reconnectScheduled = true;
			shouldSchedule = true;
		}
	}

	if (shouldSchedule)
		mg_timer_add(m_mgr, kReconnectDelayMs, MG_TIMER_ONCE | MG_TIMER_AUTODELETE, &MQTTClient::ReconnectTimerCallback, this);
}

void MQTTClient::Reconnect()
{
	std::string url;
	std::string clientId;
	{
		std::lock_guard<std::mutex> lock(m_mutex);
		m_reconnectScheduled = false;
		if (m_stopPollThread || m_connection != nullptr || m_url.empty() || m_clientId.empty())
			return;

		url = m_url;
		clientId = m_clientId;
		m_lastError.clear();
		m_state = State::Reconnecting;
	}

	if (!OpenConnection(url, clientId))
	{
		{
			std::lock_guard<std::mutex> lock(m_mutex);
			if (m_stopPollThread)
				return;
			m_lastError = "mg_mqtt_connect failed";
			m_state = State::Failed;
		}
		ScheduleReconnect();
	}
}

void MQTTClient::ReconnectTimerCallback(void* arg)
{
	auto* self = static_cast<MQTTClient*>(arg);
	if (self != nullptr)
		self->Reconnect();
}

void MQTTClient::SubTopicImp(const std::string& topic)
{
	if (m_state != State::Connected || m_connection == nullptr)
		return;

	mg_mqtt_opts opts{};
	opts.topic = mg_str(topic.c_str());
	opts.qos = 0;
	mg_mqtt_sub(m_connection, &opts);
}

bool MQTTClient::WaitForConnectResult()
{
	std::unique_lock<std::mutex> lock(m_mutex);
	m_connectCv.wait_for(lock, kConnectTimeout, [this]()
		{
			return m_state != State::Connecting;
		});
	return m_state == State::Connected;
}

bool MQTTClient::TopicMatches(const std::string& filter, const std::string& topic)
{
	if (filter == topic)
		return true;

	size_t fi = 0;
	size_t ti = 0;
	while (fi < filter.size() && ti < topic.size())
	{
		if (filter[fi] == '#')
			return fi + 1 == filter.size();

		if (filter[fi] == '+')
		{
			while (ti < topic.size() && topic[ti] != '/')
				++ti;
			++fi;
		}
		else
		{
			if (filter[fi] != topic[ti])
				return false;
			++fi;
			++ti;
		}

		if (fi < filter.size() && filter[fi] == '/' && ti < topic.size() && topic[ti] == '/')
		{
			++fi;
			++ti;
		}
	}

	if (fi == filter.size() && ti == topic.size())
		return true;

	return fi + 1 == filter.size() && fi < filter.size() && filter[fi] == '#';
}

inline void MQTTClient::PuplishTopicImp(const std::string& topic, const std::string& msg)
{
	if (m_state != State::Connected || m_connection == nullptr)
		return;

	mg_mqtt_opts opts{};
	opts.topic = mg_str(topic.c_str());
	opts.message = mg_str(msg.c_str());
	opts.qos = 2;
	opts.retain = true;
	mg_mqtt_pub(m_connection, &opts);
}
