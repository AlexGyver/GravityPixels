![PROJECT_PHOTO](https://github.com/AlexGyver/GravityPixels/blob/master/proj_img.jpg)
# Математическая модель шариков на плоскости
* [Описание проекта](#chapter-0)
* [Папки проекта](#chapter-1)
* [Схемы подключения](#chapter-2)
* [Материалы и компоненты](#chapter-3)
* [Как скачать и прошить](#chapter-4)
* [FAQ](#chapter-5)
* [Полезная информация](#chapter-6)
[![AlexGyver YouTube](http://alexgyver.ru/git_banner.jpg)](https://www.youtube.com/channel/UCgtAOyEQdAyjvm9ATCi_Aig?sub_confirmation=1)

<a id="chapter-0"></a>
## Описание проекта
Матрица адресных светодиодов, имитирующая движение шариков по наклонной плоскости
- Подробности в видео: hhttps://youtu.be/dueJTClX7c4

<a id="chapter-1"></a>
## Папки
**ВНИМАНИЕ! Если это твой первый опыт работы с Arduino, читай [инструкцию](#chapter-4)**
- **libraries** - библиотеки проекта. Заменить имеющиеся версии
- **gravity_pixel_manualMatr_manualAngl_int** - прошивка для Arduino, файл в папке открыть в Arduino IDE ([инструкция](#chapter-4))
- **gravity_pixel_optimized** - новая оптимизированная прошивка, расчёт на 30% быстрее и другие фишки
- **MPU6050_calibration** - прошивка для Arduino, калибровка акселерометра. Читай ниже


<a id="chapter-2"></a>
## Схемы
![SCHEME](https://github.com/AlexGyver/GravityPixels/blob/master/scheme.jpg)

<a id="chapter-3"></a>
## Материалы и компоненты
### Ссылки оставлены на магазины, с которых я закупаюсь уже не один год
* Arduino NANO с ногами http://ali.pub/26yo7x http://ali.pub/26yo7j
* Arduino NANO без ног http://ali.pub/26yo93 http://ali.pub/26yoa9
* Макетная плата http://ali.pub/26yoc0 http://ali.pub/26yocl
* Провода для макетки http://ali.pub/26yocs http://ali.pub/26yodk
* Акселерометр http://ali.pub/28ajiz
* Матрица 16х16 RGB http://ali.pub/27l6k1  http://ali.pub/27l6l7

## Вам скорее всего пригодится
* [Всё для пайки (паяльники и примочки)](http://alexgyver.ru/all-for-soldering/)
* [Недорогие инструменты](http://alexgyver.ru/my_instruments/)
* [Все существующие модули и сенсоры Arduino](http://alexgyver.ru/arduino_shop/)
* [Электронные компоненты](http://alexgyver.ru/electronics/)
* [Аккумуляторы и зарядные модули](http://alexgyver.ru/18650/)

<a id="chapter-4"></a>
## Как скачать и прошить
* [Первые шаги с Arduino](http://alexgyver.ru/arduino-first/) - ультра подробная статья по началу работы с Ардуино, ознакомиться первым делом!
* Скачать архив с проектом
> На главной странице проекта (где ты читаешь этот текст) вверху справа зелёная кнопка **Clone or download**, вот её жми, там будет **Download ZIP**
* Установить библиотеки в  
`C:\Program Files (x86)\Arduino\libraries\` (Windows x64)  
`C:\Program Files\Arduino\libraries\` (Windows x86)
* Подключить Ардуино к компьютеру
* Запустить файл прошивки (который имеет расширение .ino)
* Настроить IDE (COM порт, модель Arduino, как в статье выше)
* Настроить что нужно по проекту
* Нажать загрузить
* Пользоваться

### Калибровка акселерометра
Нужна для того, чтобы система знала своё "горизонтальное" положение
* Прошить скетч калибровки
* Расположить акселерометр (матрицу с приклеенным акселерометром) горизонтально
* Открыть монитор порта в Arduino IDE
* Отправить любой символ
* НЕ ТЕРЕБИТЬ АКСЕЛЕРОМЕТР! НЕ ДЫШАТЬ!
* Ждать окончания калибровки (~ 10 секунд)
* Найти в логе строчку your offsets вида  -3214, -222, 1324, -2, -67, -12
* Скопировать оффсеты в скетч GravityPixels в настройки, вот так
    // оффсеты для акселерометра
    int offsets[6] = { -3214, -222, 1324, -2, -67, -12};
* Наслаждаться

## Настройки в коде
    #define PIXEL_AMOUNT 50      // число "живых" пикселей
    #define G_CONST 9.81         // ускорение свободного падения

    #define BRIGHTNESS 150        // яркость (0 - 255)

    #define MATR_X 16            // число светодиодов по х
    #define MATR_Y 16            // число светодиодов по у 
    #define MATR_X_M 160         // размер матрицы в миллиметрах х
    #define MATR_Y_M 160         // размер матрицы в миллиметрах у

    #define PIXELZISE 10         // размер пикселя мм

    #define PIN 6                // пин ленты Din
    #define GLOW 0               // свечение
    #define ALL_BLUE 0           // все синим

    #define MIN_STEP 30          // минимальный шаг интегрирования (миллисекункды)

    // оффсеты для акселерометра
    int offsets[6] = { -3214, -222, 1324, -2, -67, -12};

<a id="chapter-5"></a>
## FAQ
### Основные вопросы
В: Как скачать с этого грёбаного сайта?  
О: На главной странице проекта (где ты читаешь этот текст) вверху справа зелёная кнопка **Clone or download**, вот её жми, там будет **Download ZIP**

В: Скачался какой то файл .zip, куда его теперь?  
О: Это архив. Можно открыть стандартными средствами Windows, но думаю у всех на компьютере установлен WinRAR, архив нужно правой кнопкой и извлечь.

В: Я совсем новичок! Что мне делать с Ардуиной, где взять все программы?  
О: Читай и смотри видос http://alexgyver.ru/arduino-first/

В: Компьютер никак не реагирует на подключение Ардуины!  
О: Возможно у тебя зарядный USB кабель, а нужен именно data-кабель, по которому можно данные передавать

В: Ошибка! Скетч не компилируется!  
О: Путь к скетчу не должен содержать кириллицу. Положи его в корень диска.

В: Сколько стоит?  
О: Ничего не продаю.

### Вопросы по этому проекту

<a id="chapter-6"></a>
## Полезная информация
* [Мой сайт](http://alexgyver.ru/)
* [Основной YouTube канал](https://www.youtube.com/channel/UCgtAOyEQdAyjvm9ATCi_Aig?sub_confirmation=1)
* [YouTube канал про Arduino](https://www.youtube.com/channel/UC4axiS76D784-ofoTdo5zOA?sub_confirmation=1)
* [Мои видеоуроки по пайке](https://www.youtube.com/playlist?list=PLOT_HeyBraBuMIwfSYu7kCKXxQGsUKcqR)
* [Мои видеоуроки по Arduino](http://alexgyver.ru/arduino_lessons/)