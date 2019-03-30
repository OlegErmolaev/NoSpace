import sys
import platform
import serial
import threading
import time
from EventMaster import *

#####custom exceptions#####

class USBError(Exception):#ошибка подключения по usb(отсутствует указанный /dev/ttyUSB? порт)
    pass

class ConnectionDownFall(Exception):#ошибка краша связи
    pass

class USBImcorrectDevice(Exception):#ошибка некорректного подключения устройства
    pass

class GlobalCrash(Exception):#краш чего-то
    pass

class Joystic(threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self)#инициализация потока
        self.Joy = serial.Serial()#создание сериала
        self.magicNumber = 387#задаём магическое число(используется для проверки того, что к нам подклчена ардуина, как джойстик с нашей прошивкой)
        self.port = None#настройки сериала возможно пригодятся
        self.baudrate = None
        self.timeout = None
        self._stopping = False#состояние остановки трэда

        self.EM = EventMaster(freq=1)#
        self.EM.start()#
        
        self.name = ""#имя устройства
        self.axisNumber = None#кол-во осей
        self.buttonsNumber = None#кол-во кнопок
        self.axisBuf = dict()#буфер со значениями осей
        self.buttonsBuf = dict()#буфер со значениями кнопок
        self.buttonHandler = dict()#словарь с обработчиками
        self.axisNames = []#массив с именами осей
        self.buttonsNames = []#массив сименами кнопок

        self.disconnectCode = "9"#при дисконнекте, чтобы джойстик ушёл в поиск нового соединения

    def connect(self, path, baudrate=9600, timeout=5):#подключение указываем номер usb порта, baudrate по умолчанию 9600, timeout по умолчанию 3 секунды
        try:
            if(platform.system() == 'Windows'):
                self.Joy.port = 'COM%d' % path#указываем порт
            else:
                self.Joy.port = "/dev/ttyUSB%d" % path#указываем порт
            self.Joy.baudrate = baudrate#указываем скорость соединения
            self.Joy.timeout = timeout#задаём время ожидания

            self.port = self.Joy.port#записываем полученные значения в локальные переменные
            self.baudrate = baudrate
            self.timeout = timeout
            
            self.Joy.open()#пытаемся открыть соединение
            time.sleep(3)
            if(self.Joy.isOpen()):#если успешно создали подключение
                self.Joy.write(str(self.magicNumber).encode())#отправляем наше магическое число                
                res = False#результат проверки на магическое число
                echoNumber = self.Joy.readline()#читаем ответ
                parsedData = self.decodeData(echoNumber)#парсим данные
                if(parsedData is not None and parsedData != ''):#если строка не пустая и мы получили магическое число
                    if(int(parsedData) == self.magicNumber):
                        print("Connected to %s on baudrate:%d" % (self.port, self.baudrate))#информируем об удачном подключении
                        self.Joy.timeout = None#удаляем таймаут
                        self.Name = self.decodeData(self.Joy.readline())#получили имя
                        self.axisNumber = int(self.decodeData(self.Joy.readline()))#получили кол-во осей
                        self.buttonsNumber = int(self.decodeData(self.Joy.readline()))#получили кол-во кнопок
                        
                        for i in range(self.axisNumber):
                            axisName = self.decodeData(self.Joy.readline())#получаем имя оси
                            self.axisBuf[str(axisName)] = 0#записываем в словарь с присвоением значения 0
                            self.axisNames.append(str(axisName))#добавляем в массив имён

                        for i in range(self.buttonsNumber):
                            buttonName = self.decodeData(self.Joy.readline())#получаем имя кнопки
                            self.buttonsBuf[str(buttonName)] = 0#записываем в словарь с присвоением значения 0
                            self.buttonsNames.append(str(buttonName))#добавляем в массив имён
                        
                        res = True#всё ок
                    else:
                        raise USBImcorrectDevice#получили, но не то вызываем исключение
                if(not res):#если не получили вообще ничего
                    raise ConnectionDownFall#вызываем исключение
                
        except serial.serialutil.SerialException:#если такого устройства нет
            raise USBError("No such device connected")#вызываем исключение
            sys.exit(1)#выходим с 1 кодом
        except ConnectionDownFall:#если время вышло
            raise ConnectionDownFall("Timeout is up")#вызываем исключение
            sys.exit(2)#выходим с кодом 2
        except USBImcorrectDevice:#если получили не наше магическое число
            raise USBImcorrectDevice("This device isn't supported or can't read data frome device")#вызываем исключение
            sys.exit(3)#выходим с кодом 3
            
    def run(self):
        while not self._stopping:
            echo = self.Joy.readline()#считываем данные
            if(echo is not None):
                self.parseData(echo)#если что-то есть вызываем парсинг
            else:
                self.stop()
                raise ConnectionDownFall("Связь потеряна")#иначе вызываем исключение
            
        self.disconnect()#отключаемся
        
    def disconnect(self):
        #отправляем запрос на отключение#
        self.Joy.write(str(self.disconnectCode).encode())
        self.Joy.close()
        self.EM.exit()

    def decodeData(self, echo):
        if echo is not None:#если данные не пусты
            data = ''#создаём "пустые данные"
            for i in range(len(echo)-2):#на длине строки крому \n\r
                data += chr(echo[i])#добавляем сивол к данным
            return data#возращаем данные

    def parseData(self, echo):#парсинг
        data = self.decodeData(echo)#считываем что при прилетело
        if(data != ''):
            for i in range(self.axisNumber):#в диапозоне кол-ва осей
                name = self.axisNames[i]#считываем имя из массива
                data = data[len(name):len(data)]#обрезаем имя текущей оси от данных
                j = 0
                val = ''#значение оси
                if(i != self.axisNumber - 1):#если не последняя ось в словаре
                    while(data[j]!=self.axisNames[i+1][0]):#пока не нашли следующую ось
                        val += data[j]#добавляем в значение
                        j += 1#++
                else:
                    while(data[j] != self.buttonsNames[0][0]):#до первой кнопки
                        val += data[j]#добавляем в значение
                        j += 1#++
                        
                data = data[j:len(data)]#обрезаем данные для следующей итерации
                self.axisBuf.update({self.axisNames[i]:int(val)})#обновляем значение в словаре

            for i in range(self.buttonsNumber):#в диапозоне кол-ва кнопок
                name = self.buttonsNames[i]#считываем имя из массива
                data = data[len(name):len(data)]#обрезаем имя текущей кнопки от данных
                j = 0
                val = ''#значение кнопки
                if(i != self.buttonsNumber - 1):#если не последняя кнопка
                    while(data[j]!=self.buttonsNames[i+1][0]):#до следующей кнопки
                        val += data[j]#добавляем в значение
                        j += 1#++
                        
                    data = data[j:len(data)]#обрезаем данные для следующей итерации
                    
                else:
                    val = data#значение = остатку данных

                #####EVENT MASTER#####
                if(name in self.buttonHandler):#
                    if(int(val) != self.buttonsBuf.get(name) and int(val) == 1):#
                        handler = self.buttonHandler.get(name)#
                        handler.push()#
                                        
                self.buttonsBuf.update({self.buttonsNames[i]:int(val)})#обновляем значение в словаре
            
    def info(self):
        print("Path: %s" % self.port)#путь
        print("Name: %s" % self.Name)#имя
        buttons = ""#пустая строка для кнопок
        axis = ""#пустая строка для осей
        for i in range(self.axisNumber):
            axis += self.axisNames[i]#добавляем к строке имя оси
            if(i != self.axisNumber - 1):#если не последнее имя
                axis += ", "#добавляем запятую

        for i in range(self.buttonsNumber):
            buttons += self.buttonsNames[i]#добавляем имя к строке
            if(i != self.buttonsNumber - 1):#если не последнее имя
                buttons += ", "#добавляем запятую

        print("Axis: %s" % axis)#выводим имена осей
        print("Buttons: %s" % buttons)#выводим имена кнопок

    def getAxis(self, axis):#получение значение оси
        if(not(str(axis) in self.axisBuf)):#если такой оси нет
            self.stop()#останавливаем работу
            raise GlobalCrash("Упс... Нет такой оси")#вызываем исключение
        else:
            return self.axisBuf[axis]#возвращаем значение

    def getButton(self, button):#получение значения кнопки(в дальнейшем обработчик события нажатия?)
        if(not(str(button) in self.buttonsBuf)):#если такой кнопки нет
            self.stop()#останавливаем работу
            raise GlobalCrash("Упс... Нет такой кнопки")#вызываем исключение
        else:
            return self.buttonsBuf[button]#возвращаем значение

    def connectButton(self, buttonName, func):
        if(not(str(buttonName) in self.buttonsBuf)):#если такой кнопки нет
            self.stop()#останавливаем работу
            raise GlobalCrash("Упс... Нет такой кнопки")#вызываем исключение
        else:
            ev = EventBlock()#создаём блок события
            ev.setfun(func)#устанавливаем функцию для события
            self.buttonHandler.update({buttonName : ev})#кидаем в словарь
            self.EM.append(self.buttonHandler.get(buttonName))#добавляем в EventMaster
        
    def stop(self):
        self._stopping = True        
