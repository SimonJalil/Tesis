#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "procesamiento.h"


const int VENDOR_ID = 1155;

//const int PRODUCT_ID = 14155;
/*SENSORTILE*/
const int PRODUCT_ID = 22336;

QString directorio = "/home/simon/Documentos/FACULTAD_linux/Microcontroladores/Tesis/Qt/COM_port/Sesiones/sesion";

Cdato rx_buff;
QTextStream txt_buff;
QFile arch;

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    serial = new QSerialPort ();    // Inicializa la variable Serial
    stm32_available = false;

    // Recorre un contenedor determinado buscando una variable
    // en este caso la direccion de la variable que contiene
    // la informacion del puerto serial
        foreach (const QSerialPortInfo &serial_Info, QSerialPortInfo::availablePorts()) {

        if(serial_Info.hasVendorIdentifier() && serial_Info.hasProductIdentifier()){
            if(serial_Info.vendorIdentifier() == VENDOR_ID){
                if(serial_Info.productIdentifier() == PRODUCT_ID){
                    portname = serial_Info.portName();
                    stm32_available = true;
                }
            }
        }

//        qDebug() << "Puerto: " << serial_Info.portName();
//        portname = serial_Info.portName();
//        qDebug() << "Vendor Id: " << serial_Info.vendorIdentifier();
//        vendorId = serial_Info.vendorIdentifier();
//        qDebug() << "Product Id: " << serial_Info.productIdentifier();
//        productId = serial_Info.productIdentifier();
//        stm32_available = true;
    }
    if(stm32_available){
        stm32_init();

        QString buffer = QString("c");

        if(serial->isWritable()){
            serial->write(buffer.toStdString().c_str(), buffer.size());
            qDebug() << "Primer Stop...";
            graficar = 0;
        }

        setupRealtimeData(ui->customPlot, ui->customPlot_2);

    }
    else
        qDebug() << "Error";
    graficar = 0;
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::stm32_init()
{
    serial->setPortName(portname);
    serial->setBaudRate(QSerialPort::Baud115200);
    serial->setDataBits(QSerialPort::Data8);
    serial->setParity(QSerialPort::NoParity);
    serial->setStopBits(QSerialPort::OneStop);
    serial->setFlowControl(QSerialPort::NoFlowControl);
    serial->open(QIODevice::ReadWrite);
    connect(serial,SIGNAL(readyRead()),this,SLOT(serial_read()));
    connect(qApp,SIGNAL(aboutToQuit()),this,SLOT(quitMyApp()));
}

void MainWindow::segmentar(QString dato){
    int cant_comas;
    int indice;
    float val[10];

    cant_comas = dato.count(',');
    //qDebug() << "Cantidad de comas: " << cant_comas;

    indice = dato.indexOf(',');
    //qDebug() << "Indice de la primera coma: " << indice;

    for(char i = 0; i < cant_comas + 1; i++){
        val[(int)i] = (dato.mid(0,indice)).toFloat();
        dato = dato.mid(indice+1,-1);
        indice = dato.indexOf(',');
    }

    rx_buff.set(val[0], ACELEROMETRO, EJE_X);
    rx_buff.set(val[1], ACELEROMETRO, EJE_Y);
    rx_buff.set(val[2], ACELEROMETRO, EJE_Z);

    rx_buff.set(val[3], GIROSCOPO, EJE_X);
    rx_buff.set(val[4], GIROSCOPO, EJE_Y);
    rx_buff.set(val[5], GIROSCOPO, EJE_Z);

    rx_buff.calculoMod(GIROSCOPO);
    rx_buff.calculoMod(ACELEROMETRO);

    qDebug() << "Modulo giroscopo: " << rx_buff.getMod(GIROSCOPO);
    qDebug() << "Modulo acelerometro: " << rx_buff.getMod(ACELEROMETRO);

    if(graficar == 1){
        ui->lcdNumber->display(val[6]);
        ui->lcdNumber_2->display(val[7]);
        ui->lcdNumber_3->display(val[8]);
    }
    else{
        ui->lcdNumber->display(0);
        ui->lcdNumber_2->display(0);
        ui->lcdNumber_3->display(0);
    }
}


void MainWindow::setupRealtimeData(QCustomPlot *customPlot, QCustomPlot *customPlot_2){
    customPlot->addGraph(); // Linea azul
    customPlot->graph(0)->setPen(QPen(QColor(40, 110, 255)));

    customPlot_2->addGraph(); // Linea roja
    customPlot_2->graph(0)->setPen(QPen(QColor(255, 110, 40)));

    QSharedPointer<QCPAxisTickerTime> timeTicker(new QCPAxisTickerTime);
    timeTicker->setTimeFormat("%h:%m:%s");
    customPlot->xAxis->setTicker(timeTicker);
    customPlot->axisRect()->setupFullAxesBox();
    customPlot->yAxis->setRange(20.0, 300.0);

    customPlot_2->xAxis->setTicker(timeTicker);
    customPlot_2->axisRect()->setupFullAxesBox();
    customPlot_2->yAxis->setRange(20.0, 300.0);

    // Hacer que los ejes izquierdo e inferior transfieran sus rangos a los ejes derecho y superior:
    connect(customPlot->xAxis, SIGNAL(rangeChanged(QCPRange)), customPlot->xAxis2, SLOT(setRange(QCPRange)));
    connect(customPlot->yAxis, SIGNAL(rangeChanged(QCPRange)), customPlot->yAxis2, SLOT(setRange(QCPRange)));

    connect(customPlot_2->xAxis, SIGNAL(rangeChanged(QCPRange)), customPlot_2->xAxis2, SLOT(setRange(QCPRange)));
    connect(customPlot_2->yAxis, SIGNAL(rangeChanged(QCPRange)), customPlot_2->yAxis2, SLOT(setRange(QCPRange)));

    // Establece un timer que de forma repetida llame a MainWindow::realtimeDataSlot:
    connect(&dataTimer, SIGNAL(timeout()), this, SLOT(realtimeDataSlot()));
    dataTimer.start(0); // Interval 0 means to refresh as fast as possible
}


/**************************************************SLOTS****************************************************************/
/***********************************************************************************************************************/
void MainWindow::on_pushButton_clicked()
{
    static int cont = 0;
    //QString buffer = QString("ab\r");
    QString buffer = QString("a");

    QString path = directorio;
    path.append(QString::number(cont));
    path.append("txt");

    /*Creacion de archivo de texto*/
    arch.setFileName(path);

    if(arch.exists())
        arch.remove();

    cont++;

    arch.open(QIODevice::ReadWrite | QIODevice::Text);
    if(!arch.isOpen()){
        qDebug() << "Error: archivo no abierto";
        while(1);
    }
    txt_buff.setDevice(&arch);

    if(serial->isWritable()){
        serial->write(buffer.toStdString().c_str(), buffer.size());
        qDebug() << "Run";
        graficar = 1;
     }
}


void MainWindow::on_pushButton_2_clicked()
{
    //QString buffer = QString("cd\r");
    QString buffer = QString("c");

    arch.flush();
    arch.close();

    if(serial->isWritable()){
        serial->write(buffer.toStdString().c_str(), buffer.size());
        qDebug() << "Stop";
        graficar = 0;
    }
}


void MainWindow::serial_read()
{
    while(serial->canReadLine() && stm32_available){
        QByteArray data = serial->readLine();
        QString myString(data);
        qDebug() << myString;
        if(graficar == 1)
            txt_buff << myString;

        segmentar(myString);
    }
}


void MainWindow::realtimeDataSlot(){
    static QTime time(QTime::currentTime());

    // Calcular dos nuevos puntos de datos.
    double key = time.elapsed()/1000.0; // time elapsed since start of demo, in seconds
    static double lastPointKey = 0;
    if ((key-lastPointKey > 0.002) && graficar == 1) // at most add point every 2 ms
    {
      // Añade los datos a las lineas:
//      ui->customPlot->graph(0)->addData(key, qSin(key)+qrand()/(double)RAND_MAX*1*qSin(key/0.3843));
//      ui->customPlot->graph(1)->addData(key, qCos(key)+qrand()/(double)RAND_MAX*0.5*qSin(key/0.4364));

        ui->customPlot->graph(0)->addData(key, rx_buff.getMod(GIROSCOPO));
        ui->customPlot_2->graph(0)->addData(key, rx_buff.getMod(ACELEROMETRO));

      // Reescala el valor del eje (vertical) para encajar con los correspondientes datos:
        ui->customPlot->graph(0)->rescaleValueAxis(false,true);
        ui->customPlot_2->graph(0)->rescaleValueAxis(false,true);
        lastPointKey = key;
    }

    // Hacer que el rango del eje se desplace con los datos (a un tamaño de rango constante de 8):
    ui->customPlot->xAxis->setRange(key, 8, Qt::AlignRight);
    ui->customPlot->replot();

    ui->customPlot_2->xAxis->setRange(key, 8, Qt::AlignRight);
    ui->customPlot_2->replot();

    // Calcula los frames por segundo
    static double lastFpsKey;
    static int frameCount;
    ++frameCount;
    if (key-lastFpsKey > 2) // promedio fps sobre 2 segundos
    {
      ui->statusbar->showMessage(
            QString("%1 FPS, Total Data points: %2")
            .arg(frameCount/(key-lastFpsKey), 0, 'f', 0)
            .arg(ui->customPlot->graph(0)->data()->size())//+ui->customPlot->graph(1)->data()->size())
            , 0);

      ui->statusbar->showMessage(
            QString("%1 FPS, Total Data points: %2")
            .arg(frameCount/(key-lastFpsKey), 0, 'f', 0)
            .arg(ui->customPlot_2->graph(0)->data()->size())//+ui->customPlot->graph(1)->data()->size())
            , 0);


      lastFpsKey = key;
      frameCount = 0;
    }
}


void MainWindow::quitMyApp(){
    QString buffer = QString("c");

    if(serial->isWritable()){
        serial->write(buffer.toStdString().c_str(), buffer.size());
        qDebug() << "Stop";
        graficar = 0;
    }


    //Cerrar archivo
    arch.flush();
    arch.close();

    //serial->clear();
    serial->close();

    qDebug() << "\nPuerto cerrado\n";
}
