class Conv(Module):
  __parameters__ = []
  __buffers__ = []
  training : bool
  _is_full_backward_hook : Optional[bool]
  conv : __torch__.torch.nn.modules.conv.Conv2d
  act : __torch__.utils.activations.SiLU
  def forward(self: __torch__.models.common.Conv,
    x: Tensor) -> Tensor:
    act = self.act
    conv = self.conv
    _0 = (act).forward((conv).forward(x, ), )
    return _0
class Concat(Module):
  __parameters__ = []
  __buffers__ = []
  training : bool
  _is_full_backward_hook : Optional[bool]
  def forward(self: __torch__.models.common.Concat,
    argument_1: Tensor,
    argument_2: Tensor,
    argument_3: Tensor,
    argument_4: Tensor) -> Tensor:
    _1 = [argument_1, argument_2, argument_3, argument_4]
    return torch.cat(_1, 1)
class MP(Module):
  __parameters__ = []
  __buffers__ = []
  training : bool
  _is_full_backward_hook : Optional[bool]
  m : __torch__.torch.nn.modules.pooling.MaxPool2d
  def forward(self: __torch__.models.common.MP,
    argument_1: Tensor) -> Tensor:
    m = self.m
    return (m).forward(argument_1, )
class SPPCSPC(Module):
  __parameters__ = []
  __buffers__ = []
  training : bool
  _is_full_backward_hook : Optional[bool]
  cv1 : __torch__.models.common.___torch_mangle_132.Conv
  cv2 : __torch__.models.common.___torch_mangle_135.Conv
  cv3 : __torch__.models.common.___torch_mangle_138.Conv
  cv4 : __torch__.models.common.___torch_mangle_141.Conv
  m : __torch__.torch.nn.modules.container.ModuleList
  cv5 : __torch__.models.common.___torch_mangle_147.Conv
  cv6 : __torch__.models.common.___torch_mangle_150.Conv
  cv7 : __torch__.models.common.___torch_mangle_153.Conv
  def forward(self: __torch__.models.common.SPPCSPC,
    argument_1: Tensor) -> Tensor:
    cv7 = self.cv7
    cv2 = self.cv2
    cv6 = self.cv6
    cv5 = self.cv5
    m = self.m
    _2 = getattr(m, "2")
    m0 = self.m
    _1 = getattr(m0, "1")
    m1 = self.m
    _0 = getattr(m1, "0")
    cv4 = self.cv4
    cv3 = self.cv3
    cv1 = self.cv1
    _3 = (cv3).forward((cv1).forward(argument_1, ), )
    _4 = (cv4).forward(_3, )
    _5 = [_4, (_0).forward(_4, ), (_1).forward(_4, ), (_2).forward(_4, )]
    input = torch.cat(_5, 1)
    _6 = (cv6).forward((cv5).forward(input, ), )
    input0 = torch.cat([_6, (cv2).forward(argument_1, )], 1)
    return (cv7).forward(input0, )
class RepConv(Module):
  __parameters__ = []
  __buffers__ = []
  training : bool
  _is_full_backward_hook : Optional[bool]
  act : __torch__.torch.nn.modules.activation.SiLU
  rbr_reparam : __torch__.torch.nn.modules.conv.___torch_mangle_281.Conv2d
  def forward(self: __torch__.models.common.RepConv,
    argument_1: Tensor) -> Tensor:
    act = self.act
    rbr_reparam = self.rbr_reparam
    _6 = (act).forward((rbr_reparam).forward(argument_1, ), )
    return _6
class ImplicitA(Module):
  __parameters__ = ["implicit", ]
  __buffers__ = []
  implicit : Tensor
  training : bool
  _is_full_backward_hook : Optional[bool]
class ImplicitM(Module):
  __parameters__ = ["implicit", ]
  __buffers__ = []
  implicit : Tensor
  training : bool
  _is_full_backward_hook : Optional[bool]
