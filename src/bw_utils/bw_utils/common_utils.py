import time
from collections import deque
from typing import Optional

class FrequencyMeasurer:
    def __init__(self):
        self._frequency: float = 0.0  # unit: [Hz]
        self._interval: int = 0  # unit: [ms]
        self._first: bool = True
        self._last_time: float = time.time()
        # 网络质量评估相关
        self._interval_history: deque = deque()  # 存储 (timestamp, interval) 的历史记录
        self._quality_window_seconds: float = 5.0  # 默认评估窗口：5秒
        self._quality_threshold_ms: int = 500  # 默认间隔阈值：500ms
        self._quality_max_violations: int = 3  # 默认最大违规次数：3次
        self._no_message_timeout_seconds: float = 2.0  # 默认无消息超时：2秒
        
    def measure_frequency_interval(self) -> bool:
        """
        测量频率和间隔
        
        Returns:
            bool: 如果是第一次调用返回 True，否则返回 False
        """
        now = time.time()
        
        if self._first:
            self._first = False
        else:
            # 计算间隔 (秒转毫秒)
            interval_seconds = now - self._last_time
            self._interval = int(interval_seconds * 1000)  # 转换为毫秒
            
            # 计算频率 (Hz)
            if self._interval > 0:
                self._frequency = 1000.0 / self._interval
            else:
                self._frequency = 0.0
                
            # 记录间隔历史（用于网络质量评估）
            self._interval_history.append((now, self._interval))
            
            # 清理过期的历史记录
            self._cleanup_history(now)
        
        self._last_time = now
        return self._first
    
    def set_quality_params(self, window_seconds: float = 5.0, 
                          threshold_ms: int = 500, 
                          max_violations: int = 3,
                          no_message_timeout_seconds: float = 2.0):
        """
        设置网络质量评估参数
        
        Args:
            window_seconds: 评估时间窗口（秒）
            threshold_ms: 间隔阈值（毫秒）
            max_violations: 最大违规次数
            no_message_timeout_seconds: 无消息超时时间（秒）
        """
        self._quality_window_seconds = window_seconds
        self._quality_threshold_ms = threshold_ms
        self._quality_max_violations = max_violations
        self._no_message_timeout_seconds = no_message_timeout_seconds
    
    def evaluate_connection_quality(self, 
                                  window_seconds: Optional[float] = None,
                                  threshold_ms: Optional[int] = None,
                                  max_violations: Optional[int] = None,
                                  no_message_timeout_seconds: Optional[float] = None) -> bool:
        """
        评估网络连接质量
        
        Args:
            window_seconds: 评估时间窗口（秒），None则使用默认值
            threshold_ms: 间隔阈值（毫秒），None则使用默认值  
            max_violations: 最大违规次数，None则使用默认值
            no_message_timeout_seconds: 无消息超时时间（秒），None则使用默认值
            
        Returns:
            bool: True表示连接质量差（违规次数超标或无消息超时），False表示连接质量良好
        """
        # 使用传入参数或默认值
        window = window_seconds or self._quality_window_seconds
        threshold = threshold_ms or self._quality_threshold_ms
        max_viol = max_violations or self._quality_max_violations
        timeout = no_message_timeout_seconds or self._no_message_timeout_seconds
        
        current_time = time.time()
        
        # 检查是否超时（距离上次调用超过设定时间）
        time_since_last_message = current_time - self._last_time
        if time_since_last_message > timeout:
            return True  # 无消息超时，连接质量差
        
        # 检查时间窗口内的违规次数
        cutoff_time = current_time - window
        violation_count = 0
        
        for timestamp, interval in self._interval_history:
            if timestamp >= cutoff_time and interval > threshold:
                violation_count += 1
        
        # 如果违规次数超过阈值，返回True（连接质量差）
        return violation_count >= max_viol
    
    def get_quality_stats(self, window_seconds: Optional[float] = None) -> dict:
        """
        获取网络质量统计信息
        
        Args:
            window_seconds: 统计时间窗口（秒）
            
        Returns:
            dict: 包含统计信息的字典
        """
        window = window_seconds or self._quality_window_seconds
        current_time = time.time()
        cutoff_time = current_time - window
        
        # 检查是否超时
        time_since_last_message = current_time - self._last_time
        is_timeout = time_since_last_message > self._no_message_timeout_seconds
        
        # 收集时间窗口内的数据
        recent_intervals = []
        violation_count = 0
        
        for timestamp, interval in self._interval_history:
            if timestamp >= cutoff_time:
                recent_intervals.append(interval)
                if interval > self._quality_threshold_ms:
                    violation_count += 1
        
        if not recent_intervals:
            return {
                'sample_count': 0,
                'avg_interval_ms': 0,
                'max_interval_ms': 0,
                'min_interval_ms': 0,
                'violation_count': 0,
                'violation_rate': 0.0,
                'time_since_last_message_seconds': time_since_last_message,
                'is_timeout': is_timeout,
                'is_poor_quality': is_timeout  # 无数据时，主要看是否超时
            }
        
        is_poor_quality = (violation_count >= self._quality_max_violations) or is_timeout
        
        return {
            'sample_count': len(recent_intervals),
            'avg_interval_ms': sum(recent_intervals) / len(recent_intervals),
            'max_interval_ms': max(recent_intervals),
            'min_interval_ms': min(recent_intervals),
            'violation_count': violation_count,
            'violation_rate': violation_count / len(recent_intervals),
            'time_since_last_message_seconds': time_since_last_message,
            'is_timeout': is_timeout,
            'is_poor_quality': is_poor_quality
        }
    
    def _cleanup_history(self, current_time: float):
        """
        清理过期的历史记录，只保留评估窗口内的数据
        """
        cutoff_time = current_time - self._quality_window_seconds
        while self._interval_history and self._interval_history[0][0] < cutoff_time:
            self._interval_history.popleft()
    
    @property
    def frequency(self) -> float:
        """获取频率 (Hz)"""
        return self._frequency
    
    @property 
    def interval(self) -> int:
        """获取间隔 (ms)"""
        return self._interval
    
    @property
    def time_since_last_message(self) -> float:
        """获取距离上次消息的时间（秒）"""
        return time.time() - self._last_time