"""Read-only /clock discovery guard: publisher count is separate from identity."""
class ClockGuard:
    def __init__(self):
        self.snapshots = []
        self.warned_gids = set()
        self.playback_started = None
        self.last_stamp = None
        self.last_advance_wall = None
        self.advances = 0

    def observe(self, publishers, wall_time):
        # GIDs identify endpoints; a name resolving must not create a second source.
        endpoints = {p['gid']: p for p in publishers}
        count = len(endpoints)
        event = {'wall_time': wall_time, 'count': count,
                 'publishers': list(endpoints.values()), 'state': 'zero',
                 'conflict': None, 'warning': None}
        if count > 1:
            event.update(state='multiple', conflict='Multiple active /clock publishers')
        elif count == 1:
            p = next(iter(endpoints.values()))
            name = p['node']
            if name in ('', '_NODE_NAME_UNKNOWN_', None):
                event['state'] = 'one_unresolved'
                if p['gid'] not in self.warned_gids:
                    event['warning'] = 'One /clock publisher has unresolved node identity; continuing count/clock checks'
                    self.warned_gids.add(p['gid'])
            elif name == 'rosbag2_player':
                event['state'] = 'one_expected'
            else:
                event.update(state='one_unexpected', conflict='Unexpected /clock source: '+name)
        self.snapshots.append(event)
        return event

    def start_playback(self, wall_time):
        self.playback_started = wall_time
        self.last_stamp = None
        self.last_advance_wall = None
        self.advances = 0

    def clock_received(self, stamp, wall_time):
        if self.playback_started is None:
            return
        if self.last_stamp is not None and stamp > self.last_stamp:
            self.advances += 1
            self.last_advance_wall = wall_time
        self.last_stamp = stamp

    def check_advancing(self, wall_time):
        if self.playback_started is None:
            return
        # Allow the existing 3 s player delay plus discovery/startup time.
        if self.last_advance_wall is None:
            if wall_time-self.playback_started > 12:
                raise RuntimeError('/clock did not advance within 12 wall seconds of player launch')
        elif wall_time-self.last_advance_wall > 2:
            raise RuntimeError('/clock stopped advancing for more than 2 wall seconds during playback')

    def passed(self):
        return (self.advances > 0 and any(e['count'] == 1 for e in self.snapshots)
                and not any(e['conflict'] for e in self.snapshots))
