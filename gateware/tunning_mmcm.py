from migen import *
from migen.genlib.cdc import MultiReg

from litex.gen import *
from litex.soc.cores.clock import S7PLL, S7MMCM

class TunningMMCM(S7MMCM):
    def __init__(self,
                 cd_psclk,
                 cd_sys,
                 freq_ps,
                 ctrl_size=16,
                 center_ratio=1.0, # ratio of the neutral control
                 div_n=0, # divide the tunning range and multiply the resolution by 2**div_n
                 speedgrade=-3,
                 forced_vco_freq=None):
        self.ctrl_data = Signal(ctrl_size)
        self.ctrl_load = Signal()

        super().__init__(speedgrade)

        vco_freq = 1e9 # assume 1 GHz
        if forced_vco_freq is not None:
            vco_freq = forced_vco_freq
            self.vco_freq_range = (forced_vco_freq, forced_vco_freq)

        self.expose_dps(cd_psclk, with_csr=False)
        self.params.update(
                p_CLKOUT0_USE_FINE_PS = "TRUE",
                p_CLKOUT1_USE_FINE_PS = "TRUE",
                p_CLKOUT2_USE_FINE_PS = "TRUE",
                p_CLKOUT3_USE_FINE_PS = "TRUE",
                p_CLKOUT4_USE_FINE_PS = "TRUE",
                p_CLKOUT5_USE_FINE_PS = "TRUE",
                p_CLKOUT6_USE_FINE_PS = "TRUE",
                )

        self.ps_step = 1/vco_freq/56 # size of one phase shift
        self.max_freq_shift_vco = freq_ps/16*self.ps_step / (1/vco_freq)
        self.max_freq_shift_ratio = 1 + self.max_freq_shift_vco / vco_freq
        self.min_freq_shift_ratio = 1 / self.max_freq_shift_ratio
        print("freq shift ratios :", self.min_freq_shift_ratio, self.max_freq_shift_ratio)
        assert center_ratio >= self.min_freq_shift_ratio and center_ratio <= self.max_freq_shift_ratio

        self.scaled_cmd_size = ctrl_size + div_n

        self.scaled_cmd = Signal(self.scaled_cmd_size)
        neutral = 1 << (ctrl_size - 1)
        scaled_neutral = 1 << (self.scaled_cmd_size - 1)
        center_cmd = int(scaled_neutral * (center_ratio - self.min_freq_shift_ratio) /
                         (1 - self.min_freq_shift_ratio))
        print("center_cmd", center_cmd)

        sync_sys = getattr(self.sync, cd_sys)
        sync_sys += If(self.ctrl_load,
                                self.scaled_cmd.eq(center_cmd + self.ctrl_data - neutral))

        self.scaled_cmd_abs = Signal(self.scaled_cmd_size - 1)
        sync_sys += If(self.scaled_cmd >= scaled_neutral,
                                self.psincdec.eq(1),
                                self.scaled_cmd_abs.eq(self.scaled_cmd - scaled_neutral)
                         ).Else(
                                self.psincdec.eq(0),
                                self.scaled_cmd_abs.eq(scaled_neutral - self.scaled_cmd))

        self.scaled_cmd_psclk = Signal(self.scaled_cmd_size - 1)
        self.specials += MultiReg(
                i = self.scaled_cmd_abs,
                o = self.scaled_cmd_psclk,
                odomain = cd_psclk,
                )

        sync_ps = getattr(self.sync, cd_psclk)
        self.acc_size = self.scaled_cmd_size + 3
        self.acc = Signal(self.acc_size)
        self.acc_old = Signal(self.acc_size)
        sync_ps += self.acc.eq(self.acc + self.scaled_cmd_psclk)
        sync_ps += self.acc_old.eq(self.acc)
        sync_ps += self.psen.eq(self.acc_old > self.acc)
