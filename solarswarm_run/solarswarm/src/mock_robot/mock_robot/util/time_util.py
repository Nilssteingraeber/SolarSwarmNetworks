from datetime import datetime
from zoneinfo import ZoneInfo

class TimeUtil(object):
    @classmethod
    def get_timestamp(cls) -> int:
        # returns utc timestamp as int
        return int(datetime.now(ZoneInfo("Europe/Berlin")).timestamp())

    @classmethod
    def get_datetime_f(cls) -> str:
        # returns month, day, and time as such: 'Aug 27, Wed 11:24:54'
        return datetime.now(ZoneInfo("Europe/Berlin")).strftime('%b %d, %a %H:%M:%S')
