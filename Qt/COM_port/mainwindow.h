#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <qserialport.h>
#include <qserialportinfo.h>
#include <qdebug.h>
#include <QTimer>
#include "qcustomplot.h"
#include <qtextstream.h>
#include <qfile.h>

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

    void setupRealtimeData(QCustomPlot *customPlot, QCustomPlot *customPlot_2);

private slots:
    void on_pushButton_clicked();

    void on_pushButton_2_clicked();

    void serial_read();

    //void makePlot();

    void realtimeDataSlot();

    void quitMyApp();

private:
    Ui::MainWindow *ui;
    QSerialPort *serial;                // Puerto serial
    QTimer dataTimer;                   // Timer para graficar datos
    QString portname;                   // Nombre de puerto COM
    quint16 vendorId;                   // VendorID
    quint16 productId;                  // ProductID
    bool stm32_available;               // Flag de habilitacion de puerto
    void stm32_init(void);              // Método de incializacion de puerto
    bool graficar;                      // Flag para graficar.
    void segmentar(QString dato);    // Funcion para segmentar los datos recibidos
};
#endif // MAINWINDOW_H
