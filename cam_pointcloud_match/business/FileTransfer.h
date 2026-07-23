#pragma once

#include <memory>

#include <QObject>
#include <QString>
#include <QElapsedTimer>

#include "common/data_trans/HttpClient.h"
#include "common/data_trans/MQTTClient.h"

class FileTransfer : public QObject
{
	Q_OBJECT
public:
	enum FileTransferResult
	{
		Success = 0,
		Failure = 1,
		Rejected = 2,
		Canceled = 3
	};

	FileTransfer();
	virtual ~FileTransfer();

	bool Start(const QString& serverIP);
	void Stop();
	void AnswerFileTransfer(const QString& url, bool accept, const QString& fileSavePath);

signals:
	void fileTransferCome(const QString& url, const QString& displayName);
	void fileTransferFinish(const QString& url);
	void fileTransferProcess(const QString& url, qint64 bytesReceived, qint64 writeSize, qint64 totalSize);
	void fileTransferFail(const QString& url, HttpErrorCode errorCode);

private:
	void OnFileTransfer(char data[], int dataSize);
	void PublishTransferResult(FileTransferResult result, const QString& url);
	void PublishTransferProgress(const QString& url, qint64 finishSize, qint64 totalSize);
	void ClearCurrentTransferState();

private:
	std::unique_ptr<HttpClient> m_httpClient = nullptr;
	std::unique_ptr<MQTTClient> m_mqttClient = nullptr;
	QString m_currentUrl;
	QString m_currentSaveFilePath;
	qint64 m_currentBaseSize = 0;
	QElapsedTimer m_timer;
};
