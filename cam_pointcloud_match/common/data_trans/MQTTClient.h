#pragma once

#include <atomic>
#include <condition_variable>
#include <functional>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <deque>

struct mg_mgr;
struct mg_connection;

class MQTTClient
{
public:
    using CallBack = std::function<void(char data[], int dataSize)>;

    enum class State
    {
        Stopped,
        Connecting,
        Reconnecting,
        Connected,
        Failed,
    };

    MQTTClient();
    ~MQTTClient();

    bool Connect(const std::string& url, const std::string& clienId);
    bool PuplishTopic(const std::string& topic, const std::string& msg);
    bool SubTopic(const std::string& topic, CallBack callback);
    void Disconnect();

private:
    static void EventHandler(struct mg_connection* c, int ev, void* ev_data);
    void EventHandlerImp(struct mg_connection* c, int ev, void* ev_data);
    void StartPollThread();
    void StopPollThread();
    bool OpenConnection(const std::string& url, const std::string& clientId);
    void ScheduleReconnect();
    void Reconnect();
    static void ReconnectTimerCallback(void* arg);
    void SubTopicImp(const std::string& topic);
    bool WaitForConnectResult();
    static bool TopicMatches(const std::string& filter, const std::string& topic);
	inline void PuplishTopicImp(const std::string& topic, const std::string& msg);

private:
    mg_mgr* m_mgr;
    mg_connection* m_connection;
    std::thread m_pollThread;
    std::mutex m_mutex;
    std::condition_variable m_connectCv;
    std::unordered_map<std::string, CallBack> m_callbacks;
    std::string m_url;
    std::string m_clientId;
    std::string m_lastError;
    State m_state;
    std::atomic_bool m_stopPollThread;
    bool m_reconnectScheduled;
    std::deque<std::pair<std::string, std::string>> m_pendingPublishes;
};
