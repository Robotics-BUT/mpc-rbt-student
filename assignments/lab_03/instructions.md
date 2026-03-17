# Laboratorní úloha číslo 3 - Robot Operating System

## Cíl cvičení
Cílem je si kvalitně "osahat" používání frameworku ROS 2, pochopit jeho koncepty, nástroje a možnosti.
Cvičení je koncipováno formou postupného plnění úkolů, které budou automaticky vyhodnocovány testovací Node

Čtěte pozorně zadání v rámci tohoto dokumentu.
Je doporučeno v rámci cvičení využívat dokumentaci ROS2 jak na webu, tak v helpu samotného CLI (command line interface)
Testovací Node se chová jako stavový automat. Dovolí vám tedy plnit úkoly pouze v pořadí, v jakém jsou zadány zde. Ve chvíli úspěšného splnění úlohy Node automaticky přejde do dalšího stavu a změnu zaloguje.

## Domácí příprava

### Předpoklady

* Vycházíme z předpokadů minulého cvičení. Máte tedy na školním PC připojený svůj GIT účet a stažený vlastní fork tohoto repozitáře (Bude třeba váš fork aktualizovat, aby jste dostali soubory tohoto cvičení.
* Znalosti a pochopení základních konceptů ROS 2 prezentované na přednáškách.
* Je silně doporučeno se obeznámit i s dokumentací ROS 2 na webu. Pravděpodobně ji v tomto cvičení nejednou navštívíte.
* Základní znalosti C++.
* Obratnost při práci ve více oknech terminálu najednou.

> [!IMPORTANT]  
> Zadání cvičení náročné není. Jednotlivé úkoly lze vyřešit během pár minut - není důvod tedy cvičením hnát. Dejte si čas k pochopení toho, co se na obrazovce odehrává. Bude se vám to hodit v následujících cvičeních, kde se budete muset soustředit na složitější koncepty a ROS 2 pro vás bude pouze nástroj k jejich realizaci.

## Hodnocení cvičení

> [!CAUTION]
> Na konci cvičení bude práce ohodnocena až **5 body**!

> [!CAUTION]  
> Je zakázáno jakkoliv modifikovat zdrojový kód testovací Node. Zdrojový kód máte k dispozici pouze proto, aby jste se mohli blíže seznámit s jejím vnitřním fungováním. Ostatně takto by vypadala i vaše práce s balíčky staženými od ROS2 komunity. V průběhu cvičení buďte připraveni kdykoliv vyučujícímu vysvětlit metody vašeho řešení a prezentovat příkazy, které jste využili (například příkaz **history**).

## Úkoly

Úkoly **JE** potřeba plnit v tomto pořadí. Testovací Node vnitřně funguje jako stavový automat a vyhodnocuje pouze aktuální bod zadání.

V tomto cvičení budeme pracovat v branchi **ros-intro**. Přepněte se do ní a ujistěte se, že je váš fork aktualizovaný vzhledem ke vzorovému výukovému repozitáři.

Ve Vašem ROS 2 workspace je v této branchi umístěn package **ros_intro_pkg**. Z root složky vašeho workspace ho tedy zkompilujte. Spusťte node **TestNode** z balíčku **ros_intro_pkg**. Nechte si terminál s touto node otevřený bokem, aby jste viděli jeho výstup.

1.  Vašim prvním úkolem bude publikovat jednu zprávu typu **std_msgs/msg/Bool** na topic **/student_ready**. Jste připraveni?

2.  Co takhle publikovat zpráv **100**? Nyní emulujte dálkoměr - typ **sensor_msgs/msg/Range** s rozsahem 1-10m a validní měřenou hodnotou. Senzor je umístěn ve framu **range_sensor_front**. Zjistěte si, jak je zpráva formátována a nezapomeňte - publikujte přesně 100x, jinak se vám postup anuluje.

3.  Ztratil se robot. Vysílá informace o své pozici na topic **robot_position**. Pomožte mu dostat se domů (0,0,0) - poslouchá příkazy na topicu **robot_move**. Zjistěte si jaké interfaces se na těchto topicích využívají.

4.  Nyní je ta pravá chvíle vytvořit si svoji první C++ Node. Vytvořte si nový balíček ve složce src a v něm vytvořte Node - jeho název je na vás. V Node poté vytvořte jednoduchý **publisher** (ve webové dokumentaci existuje přehledný návod) který bude na topicu **node_name** publikovat **std_msgs/msg/String** s názvem vašeho Nodu. Nezapomeňte u vašeho balíčku vyplnit i *package.xml*. Pro úspěšnou kompilaci budete muset taktéž správně nalinkovat *rclcpp* a interfaces pro *std_msgs*. Pozorně čtěte dokumentaci. Pokud vám v tomto bodě není něco 100% jasné, raději se zeptejte.
    https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Cpp-Publisher-And-Subscriber.html

5.  **Publisher** vám funguje. Do stejné Node tak přidáme i **Subscriber**. Na topicu **battery_voltage** vám nyní dvakrát za sekundu chodí měřené napětí z baterie. Tyto data vyčítejte, přepočítejte je na procenta (100% == 42V, 0% == 32V) a inhed publikujte za pomoci stejného interface na topic **battery_percentage**. 10 po sobě správně publikovaných zpráv splní bod 5.

6.  Krása ROS 2 vězí mimo jiné i v možnosti dynamické konfigurace jednotlivých Node. Na přednášce jsme mluvili o takzvaných parametrech. Zjistěte, jaké parametry obsahuje testovací Node. Pro splnění tohoto bodu parametr z konzole upravte.

7.  Parametry lze nastavovat za běhu, nebo přímo při startu Node. Do té své nyní přidejte možnost dvou Float parametrů, kterými bod 5 tohoto zadání rozšíříte o možnost nastavení maximální a minimální hodnoty napětí baterie, ze které se vypočítává její procentuální nabití. Zkuste Vaši node spustit s parametry 100% == 42V, 0% == 36V (Defaultní hodnoty parametrů nastavte na původní hodnotu z bodu 5).

8.  V tomto bodu si vyzkoušíme dva nové koncepty. Prvním z nich je launch file - jednoduchý mechanismus umožňující současné spuštění několika Node. Druhým z nich je ros2 bag - Logování ROS 2 komunikace pro sběr datasetu, nebo pozdější debuggování. V root složce vašeho workspace vytvořte složku *launch* do které vytvoříte soubor *record_battery.launch.py*. Dle dokumentace pak v launch file spusťte Vaši Node se stejnými parametry jako v bodě 7 a zároveň spusťte ros2bag. Launch file spusťte z konzole a po pár desítkách vteřin jej ukončete. Uložený ROS 2 bag si nyní můžete prohlédnout pomocí příkazu *info*, nebo spustit pomocí *play*

Zbyl vám čas? Využijte ho k experimentování. Můžete se například domluvit s kolegy, nastavit si stejné ROS_DOMAIN_ID a komunikovat napříč počítači.


> [!CAUTION]
> V průběhu cvičení nevypínejte konzoli se spuštěnou testovací Node. Ta slouží jako kontrola, že jste správně prošli všemi kroky tohoto zadání.
