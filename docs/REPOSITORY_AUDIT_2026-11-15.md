# Аудит GitHub BUSA-Monitor — 15.11.2026

Источник: GitHub API, https://github.com/e62815667-gif/BUSA-Monitor.

## Проверенные данные

| Раздел | Состояние |
|---|---|
| Ветки | 140; [полный список](BRANCH_INVENTORY_2026-11-15.md) |
| Основная ветка до начала уборки | `main`, [последний коммит на 03.11.2026](https://github.com/e62815667-gif/BUSA-Monitor/commit/01fa534bba30c80222d3d141cb149716e21dbe66) |
| Защита `main` | Не включена |
| Pull requests | 7 открытых |
| Issues | 6 открытых |
| GitHub Actions | 509 запусков |
| Рабочие процессы в `main` | 4 YAML-файла в `.github/workflows` |

## Сборки и архивы

- [Последний найденный запуск Actions](https://github.com/e62815667-gif/BUSA-Monitor/actions/runs/42094799828): 05.11.2026, ветка `feat/busa-calculator-20261105`, **success**. Это результат CI, не доказательство проверки на оборудовании.
- В этом запуске есть временный артефакт `BUSA_N16R8_RAINMAKER_RM_CALC4_WIFI1_20261105`. GitHub указал истечение срока хранения **05.12.2026**.
- [Релиз от 03.11.2026](https://github.com/e62815667-gif/BUSA-Monitor/releases/tag/busa-v14.7-unified-remote-20261103) содержит `BUSA_OTA.bin` и `BUSA_OTA.zip`. Отдельного **USB merged BIN в опубликованном релизе нет**.
- Ветка `main` не равнозначна последней собранной ветке. Рабочая версия выбирается по журналу испытаний на физическом устройстве.
- Два типа файлов для готовой прошивки: **USB merged BIN** и **OTA BIN**. До публикации каждого релиза нужно проверять наличие обоих.

## Незакрытые PR

- [#15](https://github.com/e62815667-gif/BUSA-Monitor/pull/15) — стабильность ядра RainMaker, база `main`.
- [#14](https://github.com/e62815667-gif/BUSA-Monitor/pull/14) — настройки RainMaker для телефона, база `main`.
- [#13](https://github.com/e62815667-gif/BUSA-Monitor/pull/13) — Wi-Fi / Google / TFT, база `feature/busa-v14-9-tft-settings-20261103`.
- [#12](https://github.com/e62815667-gif/BUSA-Monitor/pull/12) — настройки TFT, база `feature/busa-keyboard-14v8-nvs-20261103`.
- [#11](https://github.com/e62815667-gif/BUSA-Monitor/pull/11) — клавиатура 20 кнопок, база `feature/busa-unified-tft-status-20261103`.
- [#9](https://github.com/e62815667-gif/BUSA-Monitor/pull/9) — интерфейсы TFT, Wi-Fi и Google, база `fix/busa-local-ui-crisp-temp-icons-20261103`.
- [#6](https://github.com/e62815667-gif/BUSA-Monitor/pull/6) — диагностика N8R2, база `lab/busa-v14-14-google-readonly-audit-20261023`.

## Результаты первого этапа уборки

- Составлен полный список веток, без предположений о том, какие можно удалить.
- Составлен этот отчёт с конкретными ссылками на сборки, PR и релизы.
- Создан навигационный индекс в корневом README.

## Без изменений

Исходный код, BIN, конфигурация GitHub Actions, сами ветки, PR, Issues, релизы и защита `main` не изменены. Следующий этап требует проверить эталонные сборки, зависимости PR и согласовать перечень веток для удаления.
