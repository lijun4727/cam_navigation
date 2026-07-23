#include "workpump.h"
#include <QDebug>

WorkPump::WorkPump(QObject* parent): 
	QThread(parent),
	m_stop(false),
	m_isExecuting(false) 
{
}

WorkPump::~WorkPump()
{
	// 析构：确保后台线程已停止并释放资源
	stopPump();
}

void WorkPump::startPump(int maxTask)
{
	if (QThread::isRunning()) return;		  // 已在跑则直接返回
	stopPump();                               // 虽然线程已经停止了，还是要确保旧线程资源清理
	m_stop.store(false, std::memory_order_relaxed);
	m_maxTaskNum.store(maxTask, std::memory_order_relaxed);
	{
		QMutexLocker locker(&m_mutex);
		m_isExecuting = false;
	}
	QThread::start();
}

void WorkPump::stopPump()
{
	{
		QMutexLocker locker(&m_mutex);
		if (m_stop) return; // 已设置停止，避免重复处理
		m_stop = true;

		// 清空未执行的任务队列
		std::queue<std::function<void()>> empty;
		std::swap(m_taskQueue, empty);
	}

	// 唤醒可能正在等待的工作线程，以便其检测到停止标志并退出
	m_condition.wakeAll();

	// 阻塞等待后台线程安全结束
	this->wait();
}

bool WorkPump::isBusy() const 
{
	QMutexLocker locker(&m_mutex);
	// 忙碌条件：当前正在执行任务或队列非空
	return m_isExecuting || !m_taskQueue.empty();
}

bool WorkPump::isQueueEmpty() const
{
	QMutexLocker locker(&m_mutex);
	return m_taskQueue.empty();
}

void WorkPump::run()
{
	qDebug() << "WorkPump 后台线程已启动，线程ID:" << QThread::currentThreadId();

	while (true) {
		std::function<void()> currentTask;

		// ===== 临界区：快速从队列取出任务 =====
		{
			QMutexLocker locker(&m_mutex);

			// 若队列为空且未收到停止指令，则等待条件变量
			while (m_taskQueue.empty() && !m_stop) {
				m_isExecuting = false; // 进入等待前重置执行标记
				m_condition.wait(&m_mutex);
			}

			// 若收到停止指令，则退出循环
			if (m_stop) {
				m_isExecuting = false;
				break;
			}

			// 取出队首任务并标记为执行中
			currentTask = std::move(m_taskQueue.front());
			m_taskQueue.pop();
			m_isExecuting = true;
		}

		// ===== 非临界区：执行任务（不能持锁） =====
		// 将任务在解锁状态下执行，避免阻塞其他线程对队列的访问
		if (currentTask) {
			currentTask();
		}
	}

	qDebug() << "WorkPump 后台线程已安全退出。";
}
