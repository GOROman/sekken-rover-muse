"""バッテリー監視と自動帰還の判断。

企画書の決定事項「バッテリー低下時はチャージャーへ自動帰還」を
判断ロジックとして切り出したもの。しきい値は仮値で、実機の
バッテリー特性に合わせて到着後に調整する。
"""

from __future__ import annotations

import enum


class BatteryEvent(enum.Enum):
    NONE = "none"
    LOW = "battery_low"  # 企画書の robot->Muse イベント名に合わせる
    CRITICAL = "battery_critical"


class BatteryMonitor:
    LOW_THRESHOLD_PCT = 20.0  # これを下回ったら自動帰還を開始
    CRITICAL_THRESHOLD_PCT = 8.0  # これを下回ったら帰還を最優先（速度を落とす）
    FULL_PCT = 100.0
    CHARGED_PCT = 95.0  # ドックでここまで回復したら満充電扱い

    def __init__(self, level_pct: float = 100.0) -> None:
        self.level_pct = max(0.0, min(100.0, level_pct))
        self._low_announced = False
        self._critical_announced = False

    def drain(self, dt: float, rate_pct_per_s: float) -> BatteryEvent:
        """放電を進め、新しく発火したイベントを返す（各イベントは1回だけ）。"""
        self.level_pct = max(0.0, self.level_pct - rate_pct_per_s * dt)
        if not self._critical_announced and self.level_pct <= self.CRITICAL_THRESHOLD_PCT:
            self._critical_announced = True
            self._low_announced = True
            return BatteryEvent.CRITICAL
        if not self._low_announced and self.level_pct <= self.LOW_THRESHOLD_PCT:
            self._low_announced = True
            return BatteryEvent.LOW
        return BatteryEvent.NONE

    def charge(self, dt: float, rate_pct_per_s: float = 2.0) -> None:
        self.level_pct = min(100.0, self.level_pct + rate_pct_per_s * dt)
        if self.level_pct > self.LOW_THRESHOLD_PCT:
            self._low_announced = False
            self._critical_announced = False

    @property
    def should_return(self) -> bool:
        """自動帰還すべき残量かどうか。"""
        return self.level_pct <= self.LOW_THRESHOLD_PCT

    @property
    def is_critical(self) -> bool:
        return self.level_pct <= self.CRITICAL_THRESHOLD_PCT

    @property
    def is_charged(self) -> bool:
        return self.level_pct >= self.CHARGED_PCT
