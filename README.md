# ИТ-чемпионат «Цифровая Эра Транспорта» — 2026

[Официальная страница чемпионата](https://2026.itsrussiaforum.ru/championship/)

`MISIS x MISIS team`

Team Members:

1. **Дмитрий Коноплянников** - Backend, DevOps
2. **Виктория Гайлитис** - Backend
3. **Дарья Короленко** - Design
4. **Трофим ** - Frontend
5. **Кирилл Рыжичкин** - ML Engineer

Презентация: [тык](https://disk.yandex.ru/i/heCioDVcvBLfPw)

## Кейс "Беспилотный коридор"

> Потоковый контроллер для AV-CORRIDOR-2026-V2. На каждый JSON-пакет из stdin возвращает полный JSON-снимок в stdout. Runtime использует только стандартную библиотеку Python 3.12 и работает без сети.

## Структура

```text
solution/                 Контроллер и сохранённые параметры калибровки
tools/                    Оценка, калибровка и генерация демонстрации
tests/                    30 проверок контроллера и безопасности
demo/                     Готовая автономная демонстрация: index.html + data.js
docs/                     Архитектура, отчёт, презентация и сводка прошлой валидации
data/                     Место для внешних данных организатора (не входят в Git)
.github/workflows/ci.yml   Проверка Python и автономных тестов
Dockerfile                Сборка runtime для linux/amd64
build.sh                  Сборка Docker-образа без экспорта в архив
requirements-dev.txt      Зависимости оценки и калибровки из исходного комплекта
```

Подробности: [архитектура](docs/ARCHITECTURE.md), [отчёт](docs/REPORT.md),
[презентация](docs/presentation.pdf), [машинный контракт](docs/CONTRACT.md).

## Демонстрация

Откройте `demo/index.html` в браузере. Сервер и сеть не нужны; `data.js` должен
оставаться рядом. Для запуска через HTTP:

```bash
python3 -m http.server 8765 --bind 127.0.0.1 --directory demo
```

Адрес: http://127.0.0.1:8765/. Демонстрация показывает ранее записанный прогон,
а не запускает контроллер в браузере.

## Запуск контроллера

Используйте Python 3.12. Разместите данные организатора в `data/`, чтобы внутри
были `01_reference/`, `02_train/`, `03_public/` и `04_contract/`.
Сам набор данных в репозиторий не включён.

Все команды ниже выполняются из корня репозитория:

```bash
mkdir -p results
gzip -dc data/03_public/PUBLIC-101/packets.ndjson.gz | \
  python3.12 -u -m solution --reference data/01_reference | \
  gzip -n > results/PUBLIC-101.result.ndjson.gz
```

Контроллер читает справочники и локальные параметры из `solution/`, сохраняет
состояние между пакетами и не читает TRAIN/PUBLIC-файлы самостоятельно.
Параметр `--model-dir PATH` позволяет указать другую калибровку.

## Docker

```bash
./build.sh
mkdir -p results
gzip -dc data/03_public/PUBLIC-101/packets.ndjson.gz | \
  docker run --rm -i --platform linux/amd64 --network none \
    --read-only --tmpfs /tmp:size=64m \
    -v "$PWD/data/01_reference:/data/reference:ro" \
    corridor-solution:final | gzip -n > results/PUBLIC-101.result.ndjson.gz
```

В образ попадают только контроллер и два JSON-файла калибровки.
Готовый образ собирается из Dockerfile; TAR-архив для работы не требуется.

## Проверки и разработка

Автономные тесты не требуют сторонних библиотек:

```bash
python3.12 -m unittest discover -s tests -v
```

Без внешнего набора данных выполняются 8 тестов, ещё 22 явно пропускаются.
Для полной проверки разместите данные в `data/` или задайте
`CORRIDOR_DATA=/absolute/path/to/data`. Для проверки JSON Schema и инструментов
оценки установите зависимости из исходного комплекта:

```bash
python3.12 -m venv .venv
. .venv/bin/activate
python -m pip install -r requirements-dev.txt
python -m unittest discover -s tests -v
mkdir -p results
python -m tools.lockstep --data data --scenario PUBLIC-101 --output results/PUBLIC-101.result.ndjson.gz
python -m tools.evaluate --data data --scenario TRAIN-001 TRAIN-002 TRAIN-003 TRAIN-004 --out results/validation --validate
```

GitHub Actions выполняет автономные проверки на Python 3.12. Полная оценка
требует данных организатора. Числа в `docs/REPORT.md` и
`docs/validation-summary.json` относятся к исходной проверке решения,
а не к текущему запуску CI.

## Калибровка и пересборка демо

```bash
python -m tools.calibrate --data data --out fitted --folds
```

Для каждого из четырёх TRAIN-сценариев выполните оценку с его исключённой моделью,
подставляя соответствующий номер:

```bash
python -m tools.evaluate --data data --scenario TRAIN-003 --models fitted/folds/TRAIN-003 --out results/folds --validate
```

Когда собраны метрики всех четырёх исключённых сценариев:

```bash
python -m tools.calibrate_confidence --folds results/folds --output fitted/calibration.json
python -m tools.generate_demo --scenario PUBLIC-101
```

Последняя команда перезаписывает `demo/data.js`. PUBLIC не используется для
обучения. Проверенные параметры из `fitted/` можно явно передать через
`--model-dir fitted`; сохранённые в `solution/` параметры автоматически не меняются.
