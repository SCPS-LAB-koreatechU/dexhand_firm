"""Read-only telemetry packaging; all timestamps are host timestamps."""
import copy,time

def io_diagnostics(hand, stage):
    now=time.monotonic();status=getattr(hand,'last_status',None)
    firmware=None if status is None else dict(result=status.name, sequence=status.sequence,
        flags=status.flags, flag_names=status.flag_names(), applied_count=status.applied,
        tx_errors=status.tx_errors, rx_errors=status.rx_errors,
        decoded_monotonic_s=status.stamp, age_s=max(0,now-status.stamp))
    return dict(worker_observed_monotonic_s=now,
        stage=getattr(hand,'io_stage',stage),stats=dict(getattr(hand,'stats',{})),
        reply_timeout_s=hand.config.reply_timeout,
        last_response_age_s=None if not getattr(hand,'last_heard',0) else max(0,now-hand.last_heard),
        firmware_fault=getattr(hand,'fault',''), boot_id=getattr(hand,'boot_id',None),
        firmware_uptime_ms=getattr(hand,'_last_uptime',None),firmware_status=firmware,
        last_pose_io=copy.deepcopy(getattr(hand,'last_io_timing',None)),
        last_confirmed_command=copy.deepcopy(getattr(hand,'last_applied_command',None)),
        max_speed_fraction_s=hand.config.max_speed)
