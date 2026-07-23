// Copyright (c) Orbbec Inc. All Rights Reserved.
// Licensed under the MIT License.

#include "ui/mainwindow.h"
#include <QApplication>


#ifdef _WIN32
#include <windows.h>
#endif

int main(int argc, char* argv[]) {
#ifdef _WIN32
    SetConsoleOutputCP(CP_UTF8);
    SetConsoleCP(CP_UTF8);
    setlocale(LC_ALL, ".UTF8");
#endif

    QApplication a(argc, argv);
    // 全局样式：统一所有 QPushButton 和 QRadioButton 的背景、圆角、文字颜色和指示器
    a.setStyleSheet(R"(
        QPushButton, QRadioButton {
          background-color: rgba(0, 0, 0, 20);
          color: white;
          border-radius: 8px;
          padding: 6px 10px;
        }
        QPushButton:hover, QRadioButton:hover {
          background-color: #177DFF;
        }
        QPushButton:on, QRadioButton:on {
          background-color: #177DDC;
        }
        QPushButton:disabled, QRadioButton:disabled {
          color: #888888;
          background-color: #CCCCCC;
        }
        QRadioButton::indicator {
          width: 16px; height: 16px;
          border-radius: 8px;
          border: 1px solid #ccc;
          background: white;
        }
        QRadioButton::indicator:checked {
          background: #FFD54F;
        }
    )");

    MainWindow w;
    w.show();
    return a.exec();
}