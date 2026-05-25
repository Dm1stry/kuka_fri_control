## Computer Side 
### Проект состоит из:

- **kukafri** - код для взаимодействия с манипулятором

- **planer** - код для планирования траектории - передоставляет класс Trajectory для перемещения от точки к точки в пространстве состояний манипулятора за счет задания скорости перемещения

- **udp** - код для приема и передачи udp-сообщений. На прием и передачу сообщений выделены отдельные потоки, обмен информацией между которыми осуществляется через lockfree контейнер

- **lockfree** - код для реализации ringbuffer для межпоточного взаимодействия

### Python bindings

Pybind11-модуль собирается отдельной целью `kuka_fri_py`:

```bash
cmake -S . -B build -DBUILD_PYTHON_BINDINGS=ON
cmake --build build --target kuka_fri_py
```

Если используется локальная `deps/lib/libFRIClient.a`, ее нужно собрать с `-fPIC`, потому что Python extension является shared library. Альтернатива - передать ABI-совместимую shared-библиотеку:

```bash
cmake -S . -B build -DBUILD_PYTHON_BINDINGS=ON -DFRI_LIB=/path/to/libFRIClient.so
```

Пример использования:

```python
import kuka_fri_py as fri

controller = fri.KukaController(fri.ControlMode.JOINT_POSITION)
controller.start()
obs = controller.get_observation()
controller.stop()
```
