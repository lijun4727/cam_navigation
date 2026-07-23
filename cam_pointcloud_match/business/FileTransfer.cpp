#include "FileTransfer.h"

#include <QCoreApplication>
#include <QDir>
#include <QFileInfo>
#include <QHostInfo>
#include <QJsonDocument>
#include <QJsonObject>
#include <QMetaObject>
#include <QNetworkInterface>
#include <QStandardPaths>
#include <QSysInfo>
#include <QUrl>

namespace
{
    constexpr auto kTopicTransferEnd = "FileTransfer.End";
    constexpr auto kTopicTransferProcess = "FileTransfer.Process";
    constexpr auto kTopicTransferPrefix = "FileTrans.";

    QString FirstIpv4Address()
    {
        const auto interfaces = QNetworkInterface::allInterfaces();
        for (const auto& iface : interfaces)
        {
            if (!(iface.flags() & QNetworkInterface::IsUp)
                || !(iface.flags() & QNetworkInterface::IsRunning)
                || (iface.flags() & QNetworkInterface::IsLoopBack))
            {
                continue;
            }

            const auto entries = iface.addressEntries();
            for (const auto& entry : entries)
            {
                const auto ip = entry.ip();
                if (ip.protocol() == QAbstractSocket::IPv4Protocol && !ip.isLoopback())
                    return ip.toString();
            }
        }

        return QStringLiteral("127.0.0.1");
    }

    bool ParseDownloadJson(const QByteArray& json, QString& outUrl, QString& outDisplayName)
    {
        QJsonParseError perr;
        QJsonDocument doc = QJsonDocument::fromJson(json, &perr);
        if (perr.error != QJsonParseError::NoError || !doc.isObject()) {
            return false;
        }
        QJsonObject obj = doc.object();
        if (!obj.contains("url") || !obj.value("url").isString()) {
            return false;
        }
        if (!obj.contains("displayFileName") || !obj.value("displayFileName").isString()) {
            return false;
        }
        outUrl = obj.value("url").toString();
        outDisplayName = obj.value("displayFileName").toString();
        return true;
    }
}

FileTransfer::FileTransfer() :
    m_httpClient(std::make_unique<HttpClient>(this)),
    m_mqttClient(std::make_unique<MQTTClient>())
{
}

FileTransfer::~FileTransfer()
{
    Stop();
}

bool FileTransfer::Start(const QString& serverIP)
{
    if (m_mqttClient == nullptr)
        m_mqttClient = std::make_unique<MQTTClient>();

    QString systemName = QSysInfo::machineHostName().trimmed();
    systemName.replace(' ', '_');
    QString clientId = QStringLiteral("%1_%2").arg(systemName, FirstIpv4Address());
    QString subTopic = QStringLiteral("%1%2").arg(QLatin1String(kTopicTransferPrefix), clientId);

   const QString mqttUrl = "mqtt://" + serverIP + ":1883";
    if (!m_mqttClient->Connect(mqttUrl.toStdString(), clientId.toStdString()))
        return false;

    return m_mqttClient->SubTopic(subTopic.toStdString(),
        [this](char data[], int dataSize)
        {
            OnFileTransfer(data, dataSize);
        });
}

void FileTransfer::Stop()
{
    if (m_httpClient != nullptr && m_httpClient->IsRunning())
    {
        m_httpClient->Stop();
        if (!m_currentUrl.isEmpty())
            PublishTransferResult(FileTransferResult::Canceled, m_currentUrl);
    }

    ClearCurrentTransferState();

    if (m_mqttClient)
        m_mqttClient->Disconnect();
}

void FileTransfer::AnswerFileTransfer(
    const QString& url,
    bool accept,
    const QString& fileSavePath)
{
    const QString normalizedUrl = url.trimmed();
    if (normalizedUrl.isEmpty())
        return;

    if (!accept)
    {
        PublishTransferResult(FileTransferResult::Rejected, normalizedUrl);
        return;
    }

    if (m_httpClient == nullptr)
        m_httpClient = std::make_unique<HttpClient>(this);

    if (m_httpClient->IsRunning() && !m_currentUrl.isEmpty())
    {
        m_httpClient->Stop();
        PublishTransferResult(FileTransferResult::Canceled, m_currentUrl);
        ClearCurrentTransferState();
    }

    m_currentUrl = normalizedUrl;
    m_currentSaveFilePath = fileSavePath;
    m_currentBaseSize = QFileInfo(m_currentSaveFilePath).exists()
        ? QFileInfo(m_currentSaveFilePath).size()
        : 0;


    m_timer.start();

    const bool started = m_httpClient->Download(
        normalizedUrl,
        m_currentSaveFilePath,
        [this, normalizedUrl](qint64 bytesReceived, qint64 writeSize, qint64 totalSize, HttpErrorCode resCode)
        {
            const qint64 finishSize = m_currentBaseSize + writeSize;
            const qint64 fullSize = m_currentBaseSize + totalSize;

            switch (resCode)
            {
            case HttpErrorCode::Downloading:
                if (m_timer.elapsed() > 1000)
                {
                    emit fileTransferProcess(normalizedUrl, bytesReceived, finishSize, fullSize);
                    PublishTransferProgress(normalizedUrl, finishSize, fullSize);
					qDebug() << "Downloading:" << normalizedUrl << "progress:" << finishSize << "/" << fullSize;
                    m_timer.restart();
                }

                break;
            case HttpErrorCode::Success:
                emit fileTransferFinish(normalizedUrl);
                PublishTransferResult(FileTransferResult::Success, normalizedUrl);
                ClearCurrentTransferState();
                break;
            default:
                emit fileTransferFail(normalizedUrl, resCode);
                PublishTransferResult(FileTransferResult::Failure, normalizedUrl);
                ClearCurrentTransferState();
                break;
            }
        });

    if (!started)
    {
        emit fileTransferFail(normalizedUrl, HttpErrorCode::OpenFileFial);
        PublishTransferResult(FileTransferResult::Failure, normalizedUrl);
        ClearCurrentTransferState();
    }
}

void FileTransfer::OnFileTransfer(char data[], int dataSize)
{
    //这里推送的要求客户端下载文件的json字符串格式为{url:"下载地址", displayFileName:"文件名"}
    const QByteArray payload(data, dataSize);
    QString url;
    QString displayName;
    if (!ParseDownloadJson(payload, url, displayName)) {
        return;
    }
    emit fileTransferCome(url, displayName);
}

void FileTransfer::PublishTransferResult(FileTransferResult result, const QString& url)
{
    if (m_mqttClient == nullptr)
        return;

    QJsonObject object;
    object.insert(QStringLiteral("ret"), static_cast<int>(result));
    object.insert(QStringLiteral("url"), url);

    auto bytes = QJsonDocument(object).toJson(QJsonDocument::Compact);
    m_mqttClient->PuplishTopic(kTopicTransferEnd, bytes.toStdString());
}

void FileTransfer::PublishTransferProgress(const QString& url, qint64 finishSize, qint64 totalSize)
{
    if (m_mqttClient == nullptr)
        return;

    QJsonObject object;
    object.insert(QStringLiteral("finish_size"), finishSize);
    object.insert(QStringLiteral("total_size"), totalSize);
    object.insert(QStringLiteral("url"), url);

    auto bytes = QJsonDocument(object).toJson(QJsonDocument::Compact);
    m_mqttClient->PuplishTopic(kTopicTransferProcess, bytes.toStdString());
}

void FileTransfer::ClearCurrentTransferState()
{
    m_currentUrl.clear();
    m_currentSaveFilePath.clear();
    m_currentBaseSize = 0;
}
