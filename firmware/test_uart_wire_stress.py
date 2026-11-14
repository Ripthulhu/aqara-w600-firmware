import unittest
from unittest.mock import patch
from uart_wire_stress import WireCore,CostedCore

class WireSchedulingTests(unittest.TestCase):
    def test_pending_interrupts_are_not_idle_fast_forwarded(self):
        for rx,tick,wire,expected in [([1],False,[(100,2)],0),([],True,[],0),
             ([],False,[(100,2)],100),([],False,[],125),([],False,[(500,2)],125)]:
            m=WireCore.__new__(WireCore)
            m.reason='foreground_boundary';m.rx=rx;m.pending_tick=tick;m.wire=wire;m.virtual_us=0
            with patch.object(CostedCore,'step') as parent:
                m.step(125);parent.assert_called_once_with(expected)

if __name__=='__main__':unittest.main()
