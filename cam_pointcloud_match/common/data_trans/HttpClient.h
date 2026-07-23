#pragma once

#include <QObject>
#include <QUrl>
#include <QByteArray>
#include <functional>
#include <QFile>

class QNetworkAccessManager;
class QNetworkReply;
class QNetworkRequest;
class QTimer;

enum class HttpErrorCode
{
	Success,
	Fail,
	Timeout,
	Downloading,
	OpenFileFial,			//下载的时候，会写打开文件句柄，如果失败了，就会回调这个错误码

};

using HTTP_CALLBACK = std::function<void(const QByteArray& data, HttpErrorCode resCode)>;

/// <summary>
/// bytesReceived当前到达的字节数，m_writeSize当前已经写入的字节数，m_totalSize总字节数，resCode当前下载状态
/// </summary>
using DOWNLOAD_CALLBACK = std::function<void(qint64 bytesReceived, qint64 m_writeSize,
	qint64 m_totalSize, HttpErrorCode resCode)>;

enum class NetRequestType
{
	HttpRequest,
	Download
};

class HttpClient : public QObject
{
	Q_OBJECT
public:
	HttpClient(QObject* parent = nullptr);
	virtual ~HttpClient();

	void Get(
		const QString& url,
		HTTP_CALLBACK callback,
		int mSecond = 10000
	);
	void Post(
		const QString& url,
		const QString& postData,
		HTTP_CALLBACK callback,
		int mSecond = 10000
	);
	bool IsRunning() const;
	virtual bool Download(
		const QString& url,
		const QString& saveFilePath,
		DOWNLOAD_CALLBACK callback
	);
	void Stop();

protected slots:
	void OnReceiveReply(QNetworkReply* reply);

private:
	void Clear();
	void StopRequest();
	void InitRequest(QNetworkRequest& networkRequest);
	void InitReply(QNetworkReply* reply);

protected:
	QNetworkReply* m_reply = nullptr;
	QNetworkAccessManager* m_httpManager;
	NetRequestType m_netRequestType = NetRequestType::HttpRequest;

	//普通http请求相关
	QTimer* m_timer;
	HTTP_CALLBACK m_httpCallback;

	//下载相关
	QFile m_downFile;
	//文件总大小
	qint64 m_totalSize = 0;
	//当前已经写入的大小
	qint64 m_writenSize = 0;
	//当前已经接收的大小
	qint64 m_curReciveSize = 0;
	DOWNLOAD_CALLBACK m_downloadCallback;
	bool m_stopDownload = false;
};