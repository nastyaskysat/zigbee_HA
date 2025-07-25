# Умный дом на базе ESP32-C6 и Wiren Board с интеграцией Яндекс Алисы (Zigbee)

## Описание системы

Система обеспечивает голосовое управление устройствами умного дома через Яндекс Алису с использованием следующих компонентов:

1. **Яндекс Алиса с поддержкой Zigbee** - центральный Zigbee-хаб с голосовым управлением
2. **ESP32-C6** - Zigbee-контроллер с USBtoUART-интерфейсом
3. **Wiren Board** - промышленный контроллер для управления нагрузками

## Техническая архитектура

| Компонент | Функционал | Статус |
|-----------|------------|--------|
| Яндекс Алиса | Центр голосового управления | Реализовано |
| ESP32-C6 | Преобразование Zigbee-команд в UART, Хранение информации о состоянии регистров | Реализовано |
| UART-интерфейс | Передача команд ESP32->Wiren Board| Реализовано |
| Wiren Board | Управление релейными выходами | В разработке |
| Modbus RTU | Обмен данными между ESP32 и Wiren Board | В разработке |

## Схема подключения
```
Яндекс Алиса (Zigbee)
       ↓
    ESP32-C6 (Zigbee End Device + Modbus Slave)
       ↓ (UART/RS-485)
    Wiren Board (Modbus Master)
       ↓
    Релейные модули → Нагрузки
```

## Текущий функционал

1. Получение Zigbee-команд от Яндекс Алисы
2. Преобразование команд в UART-сообщения:
   - Формат: "CMD:EP[номер]:[ON/OFF]\r\n"
3. Отправка команд через UART на внешнее устройство
