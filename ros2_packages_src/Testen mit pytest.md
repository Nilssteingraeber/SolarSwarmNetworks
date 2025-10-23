# Unit-Test Module für Python
Python verfügt über zwei gängige Module für Unit-Tests: `unittest` und `pytest`. Während unittest eine verständlichere Syntax und mehr Assertions hat, lassen sich mit unittest keine ROS2 Nodes testen, da sie das Modul rclpy und Implementierungen der Interfaces für Messages, Services und Actions als Klassen benötigen. ROS2 Packages haben ein Verzeichnis `test/`, in diesem man Python-Skripte mit Unit-Tests hinterlegen kann. Das zum Bauen von ROS2 Packages verwendete Tool `colcon` kann mit `colcon test` dort hinterlegte Tests durchführen und stellt zur Laufzeit Abhängigkeiten bereit, jedoch funktionieren nur mit `pytest` geschriebene Tests.

```bash
    colcon test # all packages
    colcon test --packages-select <Package> [weiteres Package]
    colcon test --event-handlers console_direct+ # see errors and tests with reasons for failure
```

Referenz: https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Testing/Python.html

> Hinweis: Ein neues ROS2 Packages erhält üblicherweise bereits drei Dateien in `test/`. Diese Prüfen das Package nicht auf Funktionalität des eigenes Codes, sondern Stil. `flake8` ist besonders kritisch und gibt möglicherweise viele Code-Style-Errors an, welche zusammen stets als ein fehlgeschlagener Unit-Test zählen.

# Schreiben eines Tests mit pytest
In `test/` wird ein Python-Skript angelegt, dessen Name mit `test_` anfängt und mit `.py` endet. Einzelne Tests sind Funktionen in diesem Skript, welche mit `test_` beginnen und mit `assert` Bedingungen prüfen. Importiert werden sollte neben `pytest` auch `rclpy` zum Ausführen von Nodes, zu testende Klassen und benötigte ROS2 Interfaces. Ist ein Package im selben Workspace wie der Test gebaut, können seine Module so beispielhaft importiert werden können:
```python
    from mock_robot.util.robot_util import BaseStatusPub, Util
    from custom_interfaces.msg import RobotBattery
    from custom_interfaces.srv import RobotServiceInfo
``` 
Substruktur in `src/`
```
    .
    ├── custom_interfaces
    │   ├── CMakeLists.txt
    │   ├── LICENSE
    │   ├── msg
    │   ├── package.xml
    │   └── srv
    ├── mock_robot
    │   ├── LICENSE
    │   ├── mock_robot
    │   ├── package.xml
    │   ├── resource
    │   ├── setup.cfg
    │   ├── setup.py
    │   └── test
    ...
```

Um abstrakte Klassen oder Klassen mit abstrakten Methoden zu testen, kann eine einfache Klasse von diesen erben und abstrakte Methoden überschreiben.
```python
    class FakeStatusPub(BaseStatusPub):
        def __init__(self, nid, mac):
            BaseStatusPub.__init__(self, nid, mac)
        
        def set_activity_callback(self, request, response):
            return response
        def service_info_callback(self, request, response):
            return response
        def interface_info_callback(self, request, response):
            return response
```

Für ein Setup und Teardown verwendet `pytest` Funktionen mit dem Dekorator `@pytest.fixture`. Test-Methoden können diese als Parameter erhalten und bekommen so ihre Rückgabewerte (auch Klasseninstanzen) mitgegeben. Für ROS2 Nodes muss `yield` anstatt `return` verwendet werden, um die dekorierte Funktion nicht frühzeitig zu verlassen.
```python
    @pytest.fixture
    def node():
        rclpy.init()
        node = FakeStatusPub(NID, MAC)
        yield node
        node.destroy_node()
        rclpy.shutdown()
    
    def test_get_nid(node):
        assert node.nid == NID
```

Referenz: https://docs.pytest.org/en/stable/how-to/fixtures.html

# Tests ausführen in einem Container
Da `ros2` und `colcon` zum Testen vorausgesetzt werden, bietet es sich an, die Tests in einen Jazzy-Container durchzuführen. `*_test.dockerfile`-Dateien in `ros2_packages_src` beinhalten Anweisungen, um Unit-Tests vorhandener Packages durchzuführen.