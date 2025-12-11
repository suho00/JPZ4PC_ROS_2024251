Turtlesim P-szabályozós vezérlés és fraktálrajzolás

Ez a ROS2 csomag egy arányos szabályozót (P-controller) valósít meg a Turtlesim teknős pozícióvezérléséhez, valamint egy Koch-hópihe fraktál rajzolását.

Funkciók:
1. P-szabályozó alapú pozícióvezérlés

A `go_to(x, y)` függvény két lépésben mozgatja a teknőst:

1. A teknős ráfordul a célpont irányára.
2. Előrehalad, miközben kismértékű szögkorrekciót végez.

A lineáris és szögsebesség a hibával arányos:

* `v = Kp_linear * distance`
* `ω = Kp_angular * angle_error`

2. Fraktál rajzolása

A program rekurzívan megrajzolja a Koch-görbéket, és ezekből összeállítja a háromszög alakú hópihét.
A fraktál mélysége (`order`) állítható.

3. Launch támogatás

A csomag tartalmaz launch fájlt, amely egyszerre indítja:

* a Turtlesim grafikus alkalmazást,
* a fraktálrajzoló node-ot.
