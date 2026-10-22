# AOIM: отчёт по прямому построению B-rep

Статус: исследование в ветке `research/aoim-line-arc`. Ядро SolidCore не изменялось.
Все проверки воспроизводятся пробником

```sh
cd research/aoim-line-arc/probes/brep_probe
cargo run --offline
```

## 1. Причина остановки на `TestClosureSplitRegion`

На последней грани 90-градусной AOIM-призмы
`TestClosureSplitRegion` обнаруживает замкнутое множество faceuse и успевает
создать новую оболочку и новый регион. Затем функция возвращает

```
Unported { upstream: "SmBrep::TestClosureSplitRegion (after CalculateBoundingBox, SmBrep.cpp:27966-28583)", unit: "topology.brep_p6" }
```

Это не ошибка геометрии и не ошибка совместного построения рёбер. Разбиение
топологии уже выполнено: 8 граней, 18 рёбер, 12 вершин, по 4 edge-use на ребро.
Не выполнен только хвост `TestClosureSplitRegion`, который в C++ идёт после
`CalculateBoundingBox`:

- выбор настоящего infinite region (`FindAndSetInfiniteRegion`);
- выбор inner shell (`IsInnerShell`) и распределение faceuse, которые не
  попали в обход через edge-use, с помощью `RayFire`;
- распределение wire-рёбер и вложенных оболочек;
- удаление пустых shell/region и восстановление указателя на infinite region.

Для простой призмы без wire-рёбер, вложенных оболочек и faceuse, связанных
только через вершины, из этого хвоста реально нужен только шаг выбора
infinite region. Остальные ветки не исполняются.

## 2. Состояние C++ и Rust

C++ (`upstream/source/SMLib/src/SmBrep.cpp`, `c9c5979`):

- `SmBrep::TestClosureSplitRegion(SmEdge*, ...)` — около строки 27654;
- `SmBrep::TestClosureSplitRegion(SmFaceuse*, ...)` — около строки 27845;
- разобранный хвост — строки 27966–28583.

Проверки C++-версии для `SmFaceuse*`:

1. Если mate faceuse уже лежит в другой shell, разбиение не нужно.
2. Face с lamina edge не может разбивать регион.
3. `ClosureTraversal` проверяет, что faceuse соединён с замкнутым множеством
   соседних faceuse.
4. Создаются новая shell и новый region; faceuse, связанные через edge-use,
   переносятся в новую shell, затем обходится mate.
5. Отправляются `SM_NO_PRE_EDIT` и `SM_NO_SPLIT_IN_BREP`.
6. Считается bounding box.
7. Если старая region была infinite и запрошено тестирование, вызывается
   `FindAndSetInfiniteRegion(..., SM_VS_INFINITEREGION_SWAP, ...)`.
8. Оставшиеся faceuse распределяются через `IsInnerShell` и `RayFire`.
9. Аналогично распределяются wire-рёбра и вложенные shell.
10. Пустые shell/region удаляются, указатель infinite region восстанавливается.

Rust (`topology::brep::p6`):

- проверки, `ClosureTraversal`, создание shell/region,
  `FaceuseShellReplacement`, обход mate, уведомления и bounding box перенесены;
- после `CalculateBoundingBox` стоит явная заглушка
  `UNPORTED(topology.brep_p6)` (около `p6.rs:1324`);
- `FindAndSetInfiniteRegion` в `topology.brep_p5` также упирается в
  `topology.brep_p9`/`topology.brep_p1`;
- публичные нижнеуровневые `topology.brep::p9::find_infinite_region` и
  `topology.brep::p1::reset_infinite_region` существуют, но `p5`-обёртка их
  пока не использует.

## 3. Почему остаются 2 shell и 2 region

Это не признак двух тел и не ошибка слияния. `TestClosureSplitRegion`
разделяет два sides замкнутой поверхности:

- одна shell содержит faceuse одной стороны каждой грани;
- вторая shell содержит mate faceuse;
- одна region становится void/infinite, вторая — solid.

Для призмы после разбиения:

```
Region0: void=1, shells=1
Region1: void=0, shells=1
Shell0:  faces=8, faceuses=8, same=8, opposite=0
Shell12: faces=0, faceuses=8, same=0, opposite=8
```

Каждая из 8 граней имеет ровно один faceuse в каждой shell
(`each_face_two_distinct_shells=true`). То есть 2 shell и 2 region — ожидаемая
структура void/solid. Ошибка была бы в том, чтобы оставить флаги
неопределёнными или считать это одним телом.

Отдельно обнаружено: у новой shell список `Shell.faces` пуст (0), хотя
`get_faceuses()` возвращает 8. Список faceuse для использованных запросов
авторитетен, но это расхождение кэша `Shell.faces` нужно проверить при
завершении порта p6.

## 4. Завершение оболочки существующими API

Прямое построение через общие вершины и рёбра работает:

- `topology::brep::p9::make_shell_vertex` — общие вершины;
- `topology::brep::p3::make_wire_edge` — общие рёбра;
- `topology::brep::p6::make_face_topology` — грани из существующих edge-use;
  для уже использованного ребра создаётся следующая пара edge-use на том же
  ребре;
- `topology::edge::edgeuses` + `edgeuse::p1::vertexuse` +
  `vertexuse::vertex` — выбор edge-use по начальной вершине.

Для простой призмы дополнительная операция **не требуется**: split уже
оставляет правильные флаги и указатель:

```text
old_infinite = b.infinite_region();          // до последней грани
... make_face_topology(last_face) ...        // возвращает Unported после split
// Region0: void=1, Region1: void=0, infinite=Region0
```

`reset_infinite_region(old_infinite, InfiniteRegionOnly)` использован в
пробнике только как проверка/фиксация list head. До и после вызова значения
`infinite` и `void` идентичны. Это допустимо только при условии, что:

- старая region действительно была infinite до split;
- нет нераспределённых faceuse, wire-рёбер и вложенных shell;
- после split ровно две shell и две region, по одному faceuse каждой грани в
  каждой shell.

Для общего случая (вложенные оболочки, wire, вершины, несколько solid)
этот путь не заменяет `FindAndSetInfiniteRegion`, `IsInnerShell`, `RayFire` и
очистку пустых shell/region. Это не скрывает некорректную топологию, но и не
является полным портом p6.

## 5. Ориентация и связность

Проверено на Line–Arc призмах (R = 4, z ∈ [-1, 1]):

| случай | профиль | contour | F / E / V | area |
|---|---|---:|---:|---:|
| 90°, базовый | `[0, 2]` | 6 | 8 / 18 / 12 | 51.933998 |
| 90°, симметричный | `[-2, 2]` | 7 | 9 / 21 / 14 | 105.358356 |
| 30° | `[-2, 2]` | 7 | 9 / 21 / 14 | 88.382434 |
| 150° | `[-2, 2]` | 7 | 9 / 21 / 14 | 127.197778 |
| 179°, `line_length=500` | `[-2, 2]` | 7 | 9 / 21 / 14 | 2492.149982 |

Для всех случаев:

- у каждого ребра 4 edge-use;
- на каждом ребре 2 edge-use `Same` и 2 `Opposite`;
- каждая грань имеет по одному faceuse в void-shell и solid-shell;
- после разворота нижнего cap-loop распределение ориентаций 9/0 или 8/0, без
  случая 7/1;
- оба порядка (`caps-first`, `sides-first`) дают одинаковые счётчики и проходят
  проверки;
- split уже оставляет `Region0.void=1`, `Region1.void=0`,
  `infinite=Region0`; `reset_infinite_region` подтверждает это (before/after
  идентичны).

Для отрицательного `d0` появляется касательный участок Line–Arc, поэтому
симметричный contour содержит 7 сегментов, а призма — 9 граней. Это не
фасеточная аппроксимация: боковые участки — плоскости, дуги — цилиндры,
торцы — плоскости.

Не подтверждено численно:

- `orient_trimmed_surfaces` останавливается на
  `SmBrep::CalculateBoundingBox` (`query.brep`), поэтому outward-ориентация
  нормалей не сертифицирована;
- `brep_volume` останавливается на `SmSurface::DropCurve`
  (`intersect.surface_drop_lib_p3`), поэтому объём не сравнён. Аналитически
  ожидается `area * height = 51.933998 * 2 = 103.867996`.

## 6. Блокирующие зависимости

1. `topology.brep_p6`, хвост `TestClosureSplitRegion` после
   `CalculateBoundingBox`: `FindAndSetInfiniteRegion`, `IsInnerShell`,
   `RayFire`-распределение, очистка пустых shell/region.
2. `query.brep::CalculateBoundingBox` — блокирует `orient_trimmed_surfaces`.
3. `intersect.surface_drop_lib_p3::SmSurface::DropCurve` — блокирует
   `brep_volume`.
4. `Shell.faces` не обновляется при переносе faceuse в новую shell; список
   faceuse корректен, но кэш требует проверки.
5. `topology.brep_p4::StitchFaces` и `topology.brep_p1`/`p5` storage-gap
   остаются блокерами старого stitch-пути, но для прямого построения не нужны.

## 7. Рекомендация

1. Оставить прямое построение общих вершин/рёбер/граней как основной путь.
2. Сохранить builder и проверки простой замкнутой оболочки: 8/18/12 для
   baseline и 9/21/14 для симметричного профиля, 4 edge-use, 2 shell/2 region,
   флаги и ориентации. `reset_infinite_region` нужен только как необязательная
   проверка list head; split уже даёт корректные флаги.
3. Не обобщать этот результат на вложенные shell/wire/vertex-only случаи без
   полного p6 tail.
4. Сохранить 90° baseline как регрессионный тест. Line–Arc 30/150/179 с
   профилем `[-2,2]` уже проходят; следующий шаг — численная проверка объёма
   после разблокировки `brep_volume`, затем L/T-профили.
5. Не портировать p6 и не возвращаться к `StitchFaces` в этой ветке.
6. Передать разработчикам SolidCore точный участок p6
   (`SmBrep.cpp:27966-28583`) и расхождение `Shell.faces` как отдельные
   findings.
