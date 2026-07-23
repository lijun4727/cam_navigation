#pragma once

#include <QThread>
#include <QMutex>
#include <QWaitCondition>
#include <QString>
#include <QVariant>
#include <queue>
#include <functional>
#include <tuple>
#include <atomic>
#include <type_traits>

/**
 * @brief WorkPump：常驻后台的异步任务泵。
 * @details 该类针对 CPU 密集型任务设计，允许投递任意可调用对象及其参数，
 * 并通过 Qt 信号将运行结果（以 QVariant 封装）安全回传到主线程。
 */
class WorkPump : public QThread {
	Q_OBJECT
public:
	explicit WorkPump(QObject* parent = nullptr);
	~WorkPump();

    /**
	 * @brief 不带任务名的投递接口
	 * @tparam F 可调用对象类型（函数指针、lambda、成员函数等）
	 * @tparam Args 可调用对象的参数类型列表
	 * @param f 要在后台执行的可调用对象
	 * @param args 可调用对象的参数，会完美转发
	 * @return 本次投递的唯一任务 id，任务完成时会随 taskFinished 信号一并发送
	 * @note 函数返回类型在编译期推导，运行时会被封装为 QVariant 发回主线程
	 */
	template<typename F, typename... Args,
		typename = std::enable_if_t<!std::is_constructible_v<QString, std::decay_t<F>>>>
	quint64 postTask(F&& f, Args&&... args) 
	{
		return postTaskImpl(QString(), std::forward<F>(f), std::forward<Args>(args)...);
	}

    /**
	 * @brief 带任务名的投递接口
	 * @param taskName 任务名或业务标识，用于在 taskFinished 中区分不同任务
	 * @return 本次投递的唯一任务 id
	 */
    template<typename F, typename... Args>
	quint64 postTask(const QString& taskName, F&& f, Args&&... args)
	{
        // 转发到统一实现，避免重复代码
		return postTaskImpl(taskName, std::forward<F>(f), std::forward<Args>(args)...);
	}

	void startPump(int maxTask = 1024);

	/**
	 * @brief 跨线程安全停止任务泵（可在 UI 线程或其他控制线程调用）
	 * @details 调用后会清空未执行的任务，并阻塞等待当前正在运行的 CPU 任务彻底结束，随后安全退出线程。
	 */
	void stopPump();

	/**
	 * @brief 跨线程查询任务泵当前是否正忙
	 * @return true 代表队列中还有任务在排队，或者当前正有某个定制函数在后台疯狂计算
	 */
	bool isBusy() const;

	/**
	 * @brief 跨线程查询任务泵队列是否为空
	 * @return true 代表队列中没有任何待执行的任务
	 */
	bool isQueueEmpty() const;

signals:
    /**
	 * @brief 任务完成信号
	 * @param taskId 本次完成任务的唯一编号
	 * @param taskName 投递任务时提供的名称，若未指定则为空字符串
	 * @param result 封装任务返回值的 QVariant；若任务返回 void，则为无效 QVariant
	 */
	void taskFinished(quint64 taskId, QString taskName, QVariant result);

protected:
    /// 线程主循环（在后台 QThread 中运行）
	void run() override;

private:
	template<typename F, typename... Args>
	quint64 postTaskImpl(const QString& taskName, F&& f, Args&&... args)
	{
		using ReturnType = std::invoke_result_t<F, Args...>;
		using ResultType = std::decay_t<ReturnType>;
		const quint64 taskId = this->m_nextTaskId.fetch_add(1, std::memory_order_relaxed);

		{
			QMutexLocker locker(&this->m_mutex);
			if (this->m_stop) return 0;

			if (m_maxTaskNum > 0 && (m_taskQueue.size() >= (size_t)(m_maxTaskNum))) {
				this->m_taskQueue.pop();
			}

			this->m_taskQueue.push(
				[f = std::forward<F>(f),
				args = std::make_tuple(std::forward<Args>(args)...), this, taskId, taskName]() mutable {
					if constexpr (std::is_void_v<ReturnType>) {
						std::apply(f, std::move(args));
						emit this->taskFinished(taskId, taskName, QVariant());
					}
					else {
						auto result = std::apply(f, std::move(args));
						emit this->taskFinished(taskId, taskName, QVariant::fromValue<ResultType>(std::move(result)));
					}
				});
		}

		this->m_condition.wakeOne();
		return taskId;
	}

	void start(QThread::Priority priority = QThread::InheritPriority) = delete;

private:
	std::queue<std::function<void()>> m_taskQueue;  // 核心任务流水线队列
	mutable QMutex m_mutex;                         // 互斥锁：保护任务队列状态及执行状态
	QWaitCondition m_condition;                     // 条件变量：用于线程的零 CPU 功耗挂起与唤醒

	std::atomic<bool> m_stop;                       // 线程退出原子原子开关
	std::atomic<quint64> m_nextTaskId{ 1 };         // 全局递增任务编号，便于识别 taskFinished 对应哪个任务
	bool m_isExecuting;                             // 运行期标志：标记后台当前是否有某个定制函数正在真正在跑
	std::atomic_int m_maxTaskNum{ 0 };				// 最大任务数限制，0代表不限制
};