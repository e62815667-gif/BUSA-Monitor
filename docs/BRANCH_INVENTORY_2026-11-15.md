# Актуальный реестр веток BUSA-Monitor — 15.11.2026

**Активных веток: 22.** Было 140: первые 3 проверенные избыточные ветки удалены отдельно, затем ещё 115 архивных веток удалены после создания и проверки полного Git-архива.

## Постоянный архив

- [GitHub Release: полный архив 137 веток до массовой очистки](https://github.com/e62815667-gif/BUSA-Monitor/releases/tag/busa-git-backup-all-branches-20261115).
- [Полный Git bundle (27,2 МБ)](https://github.com/e62815667-gif/BUSA-Monitor/releases/download/busa-git-backup-all-branches-20261115/BUSA_FULL_GIT_20261115.bundle).
- В релизе сохранены `all-branches.txt` (SHA каждой из 137 веток), `archive-only.txt` (115 удалённых после архивирования), `keep.txt`, `SHA256SUMS.txt`, инструкция восстановления.
- GitHub Actions создал архив, скачал его из GitHub Release, проверил контрольные суммы и **восстановил все 137 веток в новую чистую Git-базу** для подтверждения сохранности. Затем удалил только ветки, не входившие в список сохранения.
- Первые 3 ветки были исключены до массового архива: одна была точным дубликатом сохраняемой ветки (один SHA), две имели уже объединённые PR [#8](https://github.com/e62815667-gif/BUSA-Monitor/pull/8) и [#10](https://github.com/e62815667-gif/BUSA-Monitor/pull/10).
- [Успешный отчёт автоматической очистки](https://github.com/e62815667-gif/BUSA-Monitor/actions/runs/43694280331). Временный workflow удалён после успешного завершения.

## 22 активные ветки по назначению

### Основная ветка (1)

- [`main`](https://github.com/e62815667-gif/BUSA-Monitor/tree/main) — `e52b815b24780cef299871fc6b48fec9ad9b3166`

### Основа CALC4/WIFI1 и ключевые направления (7)

- [`architecture/busa-v15-modular-monitor-20261027`](https://github.com/e62815667-gif/BUSA-Monitor/tree/architecture/busa-v15-modular-monitor-20261027) — `ac7dcfb16ad15062bcda3a6bb8934e78487050e2`
- [`feat/busa-calculator-20261105`](https://github.com/e62815667-gif/BUSA-Monitor/tree/feat/busa-calculator-20261105) — `864a576f97c6793bfa241aaf81278821d73ca744`
- [`fix/busa-ble-ota-stable-20261105`](https://github.com/e62815667-gif/BUSA-Monitor/tree/fix/busa-ble-ota-stable-20261105) — `2b7e8ceab88da06230efc77db4f1cfce14fb8cae`
- [`fix/busa-official-rainmaker-ble-20261105`](https://github.com/e62815667-gif/BUSA-Monitor/tree/fix/busa-official-rainmaker-ble-20261105) — `07c00fb17382bc8dad61595b1ada25bcd907b63b`
- [`fix/busa-rainmaker-live-telemetry-20261105`](https://github.com/e62815667-gif/BUSA-Monitor/tree/fix/busa-rainmaker-live-telemetry-20261105) — `46baec115f91e2637c2adf493e9449093700c9d1`
- [`fix/busa-rainmaker-telemetry-20261105`](https://github.com/e62815667-gif/BUSA-Monitor/tree/fix/busa-rainmaker-telemetry-20261105) — `ee6e659eacc2fd917b7c5f11b3a756c4b67606fa`
- [`fix/busa-wifi-single-scanner-driver-20261105`](https://github.com/e62815667-gif/BUSA-Monitor/tree/fix/busa-wifi-single-scanner-driver-20261105) — `82a6925c85f6e2a91bf4c84b4abe72d7f07180c5`

### Опорные старые версии (5)

- [`fix/busa-v14-7-keypad-persist-backup-20261103`](https://github.com/e62815667-gif/BUSA-Monitor/tree/fix/busa-v14-7-keypad-persist-backup-20261103) — `349966c9e810021bd88079dd9921853bac6f3615`
- [`fix/busa-v14-9-independent-ui-sta-switch-20261103`](https://github.com/e62815667-gif/BUSA-Monitor/tree/fix/busa-v14-9-independent-ui-sta-switch-20261103) — `2c2b7c39249b4446b0c1e50092d9a78f54033f1a`
- [`release/busa-v14-7-original-n16r8-sound4-corrected-20261101`](https://github.com/e62815667-gif/BUSA-Monitor/tree/release/busa-v14-7-original-n16r8-sound4-corrected-20261101) — `0c45ecdc9282cff75792232682ad6ecd58014301`
- [`release/busa-v14-7-smooth-google-ca-bundle-20261102`](https://github.com/e62815667-gif/BUSA-Monitor/tree/release/busa-v14-7-smooth-google-ca-bundle-20261102) — `79b2eb132d591a9f537a7fda56f4a8744f542ac8`
- [`release/busa-v14-7-wifi-google-stable-20261103`](https://github.com/e62815667-gif/BUSA-Monitor/tree/release/busa-v14-7-wifi-google-stable-20261103) — `8f10dd931671439e20f737fdd435b649dea40677`

### Ветки действующих Pull Request (9)

- [`feature/busa-keyboard-14v8-nvs-20261103`](https://github.com/e62815667-gif/BUSA-Monitor/tree/feature/busa-keyboard-14v8-nvs-20261103) — `0efc09b72d3258b7afdb0c79412ae75e5d39d0ca`
- [`feature/busa-rainmaker-full-settings-20261104`](https://github.com/e62815667-gif/BUSA-Monitor/tree/feature/busa-rainmaker-full-settings-20261104) — `9524cedda5c9788132815073febf4ea84b7c0221`
- [`feature/busa-unified-tft-status-20261103`](https://github.com/e62815667-gif/BUSA-Monitor/tree/feature/busa-unified-tft-status-20261103) — `ce6445713e724b2909793db650f2f79b4e188f3f`
- [`feature/busa-v14-9-tft-settings-20261103`](https://github.com/e62815667-gif/BUSA-Monitor/tree/feature/busa-v14-9-tft-settings-20261103) — `4c97082e5aaa83a8bd41e21d77b99ee26d482bf7`
- [`feature/busa-v15-0-wifi-google-tft-20261103`](https://github.com/e62815667-gif/BUSA-Monitor/tree/feature/busa-v15-0-wifi-google-tft-20261103) — `005cc1fb57d5efbf25530acc9fe6455d12e3046b`
- [`fix/busa-local-ui-crisp-temp-icons-20261103`](https://github.com/e62815667-gif/BUSA-Monitor/tree/fix/busa-local-ui-crisp-temp-icons-20261103) — `0628686bffbb2299b3b9535d0eb9c95a812da738`
- [`fix/busa-rainmaker-core-stability-20261104`](https://github.com/e62815667-gif/BUSA-Monitor/tree/fix/busa-rainmaker-core-stability-20261104) — `a0b39afe7231a313ee0c5de2fb030e15712b463e`
- [`fix/busa-v14-17-temperature-diagnostics-20261023`](https://github.com/e62815667-gif/BUSA-Monitor/tree/fix/busa-v14-17-temperature-diagnostics-20261023) — `d650470da7f741389ea9060495474075f3c8125c`
- [`lab/busa-v14-14-google-readonly-audit-20261023`](https://github.com/e62815667-gif/BUSA-Monitor/tree/lab/busa-v14-14-google-readonly-audit-20261023) — `154a22afada9b4105da7cafbba34bd12c2ca904a`

## Восстановление одной старой ветки

1. Скачать `BUSA_FULL_GIT_20261115.bundle` из архивного релиза.
2. В локальной Git-копии выполнить:

```bash
git fetch /path/to/BUSA_FULL_GIT_20261115.bundle 'refs/heads/ИМЯ_ВЕТКИ:refs/heads/ИМЯ_ВЕТКИ'
```

3. Проверить, что восстановленный SHA совпадает с записью в `all-branches.txt`. Полный список и SHA находятся в архивном релизе.

**Важно:** архив включает только Git-исходники и историю, присутствовавшие в репозитории на момент архивирования. Он **не является исходным кодом UI16**. Два подтверждённых BIN UI16 находятся в папке Google Диска «успех буса». Подробности: [паспорт UI16](UI16_VERIFIED_2026-11-15.md).
