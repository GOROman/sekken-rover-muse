# せっけんロボ ミューズ — 制御骨格（シミュレーション段階）

RDK Challenge 2026 Winter（Fun Robotics）のプロジェクト
「せっけんロボ ミューズ」のソフトウェア骨格。

- 駆動方式: **2輪・倒立振子**（IMU による姿勢制御）
- バッテリー低下時は **チャージャーへ自動帰還**
- 頭脳: D-Robotics RDK X5（所持済み）

> ⚠️ **実機未検証**: IMU（M5Stack IMU Unit、注文 M5-115515）は到着見込み
> 2026-11-10 頃で未到着。そのため現段階の検証はすべてシミュレーション上。
> 実機での動作確認が済んだとは表記しない。

## 何が入っているか

| モジュール | 内容 |
|---|---|
| `sekken_rover/plant.py` | 2輪倒立振子の物理モデル（検証用プラント） |
| `sekken_rover/imu.py` | IMU 接続点。仮想 IMU と実機用スタブ（`M5StackImuSource`） |
| `sekken_rover/controller.py` | 倒立振子 PID 制御（速度ループ＋姿勢ループ） |
| `sekken_rover/battery.py` | バッテリー監視と自動帰還の判断（低残量 20% / 危険 8%） |
| `sekken_rover/robot.py` | 状態機械: balancing → returning → charging、転倒保護 |
| `sekken_rover/sim.py` | シミュレーション実行（バランス／バッテリー帰還の2シナリオ） |

## 動かし方

```bash
# テスト（標準ライブラリのみ、追加インストール不要）
python3 -m unittest discover -s tests -v

# 10度傾けて立て直す + 外乱からの回復
python3 -m sekken_rover.sim balance

# 残量低下 -> 自動帰還 -> ドッキング -> 充電完了まで
python3 -m sekken_rover.sim battery
```

## テストで確認していること

- 前傾・後傾のどちらにも、立て直す向きの補正出力が返る
- 初期傾き10度から姿勢が収束する／押された外乱から回復する
- 目標速度に追従して走行できる
- 残量がしきい値を下回ると `battery_low` が発火し、自動帰還→ドッキング→充電完了まで遷移する
- 残量に余裕がある間は帰還しない
- IMU は差し替え可能で、実機スタブは未実装であることが明示される

## IMU 到着後にやること

`docs/IMU_INTEGRATION.md` に接続点と調整チェックリストをまとめている。
要約: `M5StackImuSource` の実装（I2C・軸マッピング・バイアス補正・フィルタ）、
PID ゲインの実機再調整、残量しきい値の実測合わせ、転倒保護の実機確認。

## スコープ外（この骨格に含まないもの）

企画書どおり対象は「2輪倒立振子」と「バッテリー低下時の自動帰還」のみ。
人検出・追従、障害物回避、Muse 連動 API、SLAM は Stage 2 以降の別作業。
