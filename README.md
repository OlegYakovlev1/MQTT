# ESP32-S3 BME280 MQTT

Проєкт демонструє роботу **ESP32-S3** з датчиком **BME280** та передачу отриманих даних через **MQTT**.

ESP32-S3 зчитує температуру, вологість і атмосферний тиск та публікує їх у MQTT-брокер раз на секунду. Також пристрій підписується на топіки інших студентів і виводить отримані повідомлення в консоль.

## Hardware

- ESP32-S3
- BME280
- Wi-Fi мережа

## Підключення BME280

| BME280 | ESP32-S3 |
| ------ | -------- |
| VIN    | 3.3V     |
| GND    | GND      |
| SDA    | GPIO 8   |
| SCL    | GPIO 9   |

BME280 підключений через інтерфейс **I2C** з адресою `0x76`.

## MQTT

Використовується публічний MQTT-брокер:

```text
mqtt://broker.hivemq.com
```

Дані публікуються раз на секунду у власні топіки:

```text
EMB1/oyakovlev/temp
EMB1/oyakovlev/hum
EMB1/oyakovlev/pres
```

Приклад значень:

```text
EMB1/oyakovlev/temp -> 28.20
EMB1/oyakovlev/hum  -> 30.67
EMB1/oyakovlev/pres -> 1011.68
```

## Підписка

Для отримання даних інших студентів використовуються MQTT wildcard-топіки:

```text
EMB1/+/temp
EMB1/+/hum
EMB1/+/pres
```

Символ `+` дозволяє отримувати повідомлення від будь-якого студента в групі `EMB1`.

## Приклад роботи

```text
I MQTT_BME280: --- MQTT MESSAGE ---
Topic: EMB1/oyakovlev/temp
Data: 28.20

I MQTT_BME280: --- MQTT MESSAGE ---
Topic: EMB1/oyakovlev/pres
Data: 1011.68

I MQTT_BME280: --- MQTT MESSAGE ---
Topic: EMB1/oyakovlev/hum
Data: 30.67

I MQTT_BME280: T: 28.19 C | H: 30.64 % | P: 1011.70 hPa
I MQTT_BME280: MQTT data published
```

## Функціональність

- Підключення ESP32-S3 до Wi-Fi
- Робота з BME280 через I2C
- Зчитування температури
- Зчитування вологості
- Зчитування атмосферного тиску
- Підключення до MQTT-брокера
- Публікація даних раз на секунду
- Підписка на топіки інших пристроїв
- Виведення отриманих MQTT-повідомлень у консоль
- Автоматичне перепідключення Wi-Fi та MQTT
