class BasicBlock(Module):
  __parameters__ = []
  __buffers__ = []
  training : bool
  _is_full_backward_hook : Optional[bool]
  conv1 : __torch__.torch.nn.modules.conv.___torch_mangle_63.Conv2d
  bn1 : __torch__.torch.nn.modules.batchnorm.___torch_mangle_64.BatchNorm2d
  relu : __torch__.torch.nn.modules.activation.___torch_mangle_65.ReLU
  conv2 : __torch__.torch.nn.modules.conv.___torch_mangle_66.Conv2d
  bn2 : __torch__.torch.nn.modules.batchnorm.___torch_mangle_67.BatchNorm2d
  def forward(self: __torch__.torchvision.models.resnet.___torch_mangle_68.BasicBlock,
    argument_1: Tensor) -> Tensor:
    bn2 = self.bn2
    conv2 = self.conv2
    relu = self.relu
    bn1 = self.bn1
    conv1 = self.conv1
    _0 = (bn1).forward((conv1).forward(argument_1, ), )
    _1 = (conv2).forward((relu).forward(_0, ), )
    input = torch.add_((bn2).forward(_1, ), argument_1)
    return (relu).forward1(input, )