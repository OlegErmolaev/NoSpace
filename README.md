# NoSpace
Проги для управления линейкоботом

## Работоспособность
- автономное движение работает?(необходимы правки)
- передача картинки работает
- управление работает
- управление манипулятором
- управление поворотом камеры

## Изменения
- убрал костыли в виде тройной конструкции try except для отправки данных через xmlrpc
- отправка отладочной картинки автономки на socket c проверкой контрольной суммы
- все управление теперь построено на User Datagram Protocol socket => при том же коде именно из-за xmlrpc манипулятор тупил и 5 лет выходил в рабочее положение, теперь он очень быстро работает, все предыдущие функции управления восстановлены, внес правки в управление поворотм камеры(должно работать еще не тестил)

## Текущие задачи
идет интеграция кода для отправки картинrи с эндоскопа с обработкой в opencv на socket

### Из-за огромного количества точек уязвимости, диоды будут гореть постоянно во время попытки


