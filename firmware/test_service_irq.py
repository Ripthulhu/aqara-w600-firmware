"""Handler scheduling checks; actual linked handlers run in uart_wire_stress."""
import unittest
from unittest.mock import Mock

from service_thumb import ServiceCore,base,ST,ICSR,PENDING


class InterruptTests(unittest.TestCase):
    def core(self):
        m=ServiceCore.__new__(ServiceCore)
        regs={base.UC_ARM_REG_PRIMASK:0,base.UC_ARM_REG_XPSR:0x01000000,
              base.UC_ARM_REG_SP:0x20006fc0,base.UC_ARM_REG_MSP:0x20006fc0,
              base.UC_ARM_REG_PC:0x08002700,base.UC_ARM_REG_LR:0x08002501}
        m.cpu=Mock()
        m.cpu.reg_read.side_effect=lambda r:regs[r]
        m.cpu.reg_write.side_effect=lambda r,v:regs.__setitem__(r,v)
        m.cpu.context_save.side_effect=lambda:regs.copy()
        m.cpu.context_restore.side_effect=lambda saved:(regs.clear(),regs.update(saved))
        memory={ST:7,ICSR:PENDING,0xe000e100:1<<27,
                0xe000ed20:64<<24,0xe000e418:0}
        m.get=lambda a:memory[a]
        m.put=lambda a,v:memory.__setitem__(a,v)
        m.reason='foreground_boundary';m.in_handler=False;m.pending_tick=True
        m.nested_irq_count=0;m.irq_depth=0;m.max_irq_depth=0;m.ticks=0;m.uart_irqs=0
        m.vectors=[0]*44;m.vectors[15]=0x08002401;m.vectors[43]=0x08002501
        return m,regs,memory

    def test_uart_priority_and_nested_context_restore(self):
        m,regs,memory=self.core();original=regs.copy();calls=[]
        self.assertEqual((m.irq_priority(43),m.irq_priority(15)),(0,64))
        def execute(entry,budget):
            calls.append((regs[base.UC_ARM_REG_XPSR]&0x1ff,entry))
            regs[base.UC_ARM_REG_PC]=entry+2
            regs[base.UC_ARM_REG_LR]=0x12345679
            if len(calls)==1:
                m.irq_vector=43;m.reason='nested_irq_boundary'
            else:m.reason='handler_return'
        m.execute=execute
        m.handler(15)
        self.assertEqual([vector for vector,_ in calls],[15,43,15])
        self.assertEqual(regs,original)
        self.assertEqual((m.nested_irq_count,m.max_irq_depth,m.irq_depth),(1,2,0))
        self.assertEqual((m.ticks,m.uart_irqs),(1,1))
        self.assertFalse(m.pending_tick or m.in_handler)
        self.assertFalse(memory[ICSR]&PENDING)

    def test_lower_priority_cannot_preempt_uart(self):
        m,regs,_=self.core();m.in_handler=True
        regs[base.UC_ARM_REG_XPSR]=0x01000000|43
        with self.assertRaises(AssertionError):m.handler(15)

    def test_masked_handler_entry_rejected(self):
        m,regs,_=self.core();regs[base.UC_ARM_REG_PRIMASK]=1
        with self.assertRaises(AssertionError):m.handler(43)


if __name__=='__main__':unittest.main()
