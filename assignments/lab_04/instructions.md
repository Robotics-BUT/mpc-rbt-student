# Laboratorní úloha číslo 4 - Simulátor Webots

## Cíl cvičení
V minulém cvičení jste se naučili práci s jedním ze dvou nástrojů potřebných pro úspěšné splnění tohoto předmětu - ROS 2. Dnes se naučíte pracovat také se simulátorem Webots, který simuluje vstupní a výstupní chování fyzického robotu.
Projekt, ve kterém dnes začnete pracovat je takřka holý. Obsahuje pouze simulované prostředí, robotickou platformu osazenou Lidarem a balíček, který se stará o překlad ROS 2 zpráv na funkce Webots a naopak.
Cílem tohoto cvičení tedy bude dokonale porozumět struktuře projektu mpc-rbt-simulator a vaší interakcí s ním z projektu mpc-rbt-student.
Zpečetěním tohoto pochopení bude v druhé části cvičení tvorba vašeho prvního balíčku, který bude se simulovaným robotem interagovat a nastavovat jeho úhlovou a lineární rychlost.

## Domácí příprava
Je výhodné si doma vyzkoušet práci s GUI Webots (poskytuje binárky i pro Win či macOS), je ale tvořen tak, že se s ním zvládnete plně seznámit i v rámci samotného cvičení.

### Předpoklady

* Opět vycházíme z předpokladu minulých cvičení - funkční propojení s Vaším Githubovým účtem.
* Znalosti a pochopení základních konceptů ROS 2 prezentované na přednáškách.
* Základní znalosti C++.
* Obratnost při práci ve více oknech terminálu najednou.

## Hodnocení cvičení

> [!CAUTION]
> Na konci cvičení bude práce ohodnocena až **5 body**!

## Úkoly

Zkontrolujte, že máte na vašem PC správně vytvořený ROS 2 workspace. Struktura projektu by nyní měla být následující: 
```
váš_workspace
│   build
│   install  
│   log
└───src
    └───mpc-rbt-simulator
    │   │   ...
    │   │   ...
    │   │   ...
    │
    └───mpc-rbt-student
        └───src
            └───zde od dnešní hodiny pracujete
            │   ...
            │   ...
```

**mpc-rbt-student** je Váš fork studentského repozitáře, ve kterém jste doposud pracovali.
Od dnešního cvičení budete pracovat pouze v **main** branchi vašeho repozitáře.
V konzoli se přesuňte do složky mpc-rbt-simulator a pomocí následujícího příkazu se přesuňte do hlavní větve:
> git checkout main

**mpc-rbt-simulator** je nový repozitář, který si dnes naklonujete. Obsahuje simulátor, integraci s ROS 2 a předpřipravenou scénu simulované továrny.
ve správném místě vašeho workspace naklonujte následující repozitář:
> git clone git@github.com:Robotics-BUT/mpc-rbt-simulator.git

Přesuňte se do kořenové složky vašeho workspace a projekt zkompilujte. Přesvěčte se, že máte správně nakonfigurovaný skript *.bashrc*, aby byl výsledek kompilace viditelný z ROS 2 CLI.
Dle návodu v repozitáři simulátoru nyní simulátor spusťte, mělo by se Vám zobrazit grafické okno se simulovaným robotem a továrnou Webots. Není důvod nic instalovat, PC má všechny balíčky předstažené a předinstalované. Pouze spusťe launchfile uvnitř balíčku se simulátorem.

Zkontrolujte výstup konzole, zda byly všechny součásti správně spuštěny. Hledáte informaci o korektním spuštění následujících ROS 2 Node

```
Ros2Supervisor
lifecycle_manager_map_server
map_server
robot_state_publisher
tiago_base
```

Jejich správné zavedení můžete zkontrolovat i z ROS 2 CLI introspekčním příkazem, který znáte z minula.

Důležitá zpráva v konzoli je pro vás:

> Controller successfully connected to robot in Webots simulation.

Značí, že došlo k úspěšnému propojení ROS 2 s robotem v simulaci.

Otevřete si nyní GUI simulátoru. Všimněte si, kromě 3D prostředí v samotném prostředku okna, i panelů vlevo (obsahuje soupis všech objektů v aktuální scéně a jejich parametrů) a dole (obsahuje konzolový výstup Webots)

Zkuste si prokliknout jednotlivé objekty v simulaci a podívat se (nebo i měnit) na jejich parametry. Zjistíte, že pro šetření výkonu se fyzika počítá pouze tam, kde je nezbytně nutné.

Stejnou reprezentaci světa naleznete také v samotném projektu ve složce *worlds*. Prohlédněte si ji.

Na vrchu grafického rozhraní simulátoru naleznete také ovládací prvky pro změnu rychlosti simulace. Ve fialovém boxu vidíte čas od začátku simulace a násobitel, kterému říkáme *real-time-factor*. Ten značí rychlost chodu simulace jako poměr k reálnému času.

Nyní si zkuste do simulace přidat další box a umístit ho kamkoliv na podlahu.
Když pak na jakýkoliv předmět v simulaci kliknete, objeví se Vám možnost ho přesouvat.

Ve složce projektu zkuste nyní nalézt, jak dochází k propojení s ROS 2. Najděte také bod, kde do simulátoru vcházejí data o požadované rychlosti robotu. Na tomto místě naleznete také Vám jistě známý výpočet inverzní kinematiky. Implementuje tento driver také nějakou bezpečnostní funkci? Jakou?

Určitě jste také našli topic, na kterém driver poslouchá požadované rychlosti robotu, než je předá simulátoru. 
Zkuste na ní manuálně publikovat rychlosti pro robot.

Nyní si vytvoříme první Node, která bude se simulátorem komunikovat. Ve Vašem balíčku **mpc-rbt-student** jsou vytvořeny zdrojové soubory pro node *keyboard_control_node*, která bude realizovat ovládání robotu za pomocí klávesnice.

Podívejte se na následující soubory ve složkách **src** a **include**:
> include/KeyboardControl.hpp -- Obsahuje deklaraci třídy KeyboardControlNode, jejích metod a vnitřních proměnných

> src/KeyboardControl.cpp -- Obsahuje pouze implementace funkcí z hlavičkového souboru
 
> src/keyboard_control_node.cpp -- Obsahuje main funkci, která spouští KeyboardControlNode

Jak jsme si říkali na přednášce - jelikož je tato node psaná v C++, build systém *colcon* po nás požaduje také soubory *package.xml* a *CMakeLists.txt*. První z nich obsahuje metadata k celému balíčku pro indexaci, druhý z nich je jakýsi předpis pro jeho kompilaci. Pokud nebudou vaše zdrojové kódy v tomto předpisu správně uvedné, nebo tu zapomenete uvést knihovnu, kterou váš balíček využívá, projekt nikdy správně nezkompiluje.

Projděte si tedy oba soubory a zorientujte se v nich. *CMakeLists.txt* budete navštěvovat pravidelně v každém z následujících cvičení.

Čtení vstupů z klávesnice může vypadat například takto:
```
KeyboardControlNode::KeyboardControlNode(): rclcpp::Node("keyboard_control_node") {

...

// Set terminal settings to non-blocking
tcgetattr(STDIN_FILENO, &old_termios_);
struct termios new_termios = old_termios_;
new_termios.c_lflag &= ~(ICANON | ECHO);
tcsetattr(STDIN_FILENO, TCSANOW, &new_termios);

fcntl(STDIN_FILENO, F_SETFL, O_NONBLOCK);

RCLCPP_INFO(this->get_logger(), "Use Arrow Keys to control the robot. Press 'ctrl+c' to quit.");

...

}

KeyboardControlNode::~KeyboardControlNode() {
    tcsetattr(STDIN_FILENO, TCSANOW, &old_termios_);
}

void KeyboardControlNode::timerCallback() {
    geometry_msgs::msg::Twist twist{};
    char c;

    fd_set readfds;
    struct timeval timeout;
    FD_ZERO(&readfds);
    FD_SET(STDIN_FILENO, &readfds);

    timeout.tv_sec = 0;
    timeout.tv_usec = 0;

    int retval = select(STDIN_FILENO + 1, &readfds, nullptr, nullptr, &timeout);

    if (retval > 0 && FD_ISSET(STDIN_FILENO, &readfds)) {
        if (read(STDIN_FILENO, &c, 1) == 1) {
            if (c == '\033') { // ESC sequence (arrow keys)
                char seq[2];
                if (read(STDIN_FILENO, &seq, 2) != 2)
                    return;

                if (seq[0] == '[') {
                    switch (seq[1]) {
                        case 'A':
                            twist.linear.x = 0.5;  // up arrow
                            break;
                        case 'B':
                            twist.linear.x = -0.5; // down arrow
                            break;
                        case 'C':
                            twist.angular.z = -0.5; // right arrow
                            break;
                        case 'D':
                            twist.angular.z = 0.5;  // left arrow
                            break;
                    }
                }
            }

            twist_publisher_->publish(twist);
        }
    }
    // else no data was available, do nothing
}
```

Celý projekt zkompilujte a spusťte Vaši novou KeyboardControlNode. Otestujte, zda se Vám robot v simulaci pohybuje.

> [!CAUTION]
> Pro získání plného počtu bodů ze cvičení implementujte ve Vaší Node parametr, který nastavuje rychlost pohybu robotu v simulaci.
