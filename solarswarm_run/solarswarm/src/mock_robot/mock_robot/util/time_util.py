from datetime import datetime, timezone

class TimeUtil(object):
    @classmethod
    def get_timestamp(cls) -> int:
        # returns utc timestamp as int
        return int(datetime.now(timezone.utc).timestamp())

    @classmethod
    def get_datetime_f(cls) -> str:
        # returns month, day, and time as such: 'Aug 27, Wed 11:24:54'
        return datetime.now().strftime('%b %d, %a %H:%M:%S')
