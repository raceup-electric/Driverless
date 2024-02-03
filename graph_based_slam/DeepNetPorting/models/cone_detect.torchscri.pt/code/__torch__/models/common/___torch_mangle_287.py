class RepConv(Module):
  __parameters__ = []
  __buffers__ = []
  training : bool
  _is_full_backward_hook : Optional[bool]
  act : __torch__.torch.nn.modules.activation.___torch_mangle_285.SiLU
  rbr_reparam : __torch__.torch.nn.modules.conv.___torch_mangle_286.Conv2d
  def forward(self: __torch__.models.common.___torch_mangle_287.RepConv,
    argument_1: Tensor) -> Tensor:
    act = self.act
    rbr_reparam = self.rbr_reparam
    _0 = (act).forward((rbr_reparam).forward(argument_1, ), )
    return _0
