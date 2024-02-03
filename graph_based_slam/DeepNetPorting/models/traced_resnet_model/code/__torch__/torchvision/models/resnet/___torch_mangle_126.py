class ResNet(Module):
  __parameters__ = []
  __buffers__ = []
  training : bool
  _is_full_backward_hook : Optional[bool]
  conv1 : __torch__.torch.nn.modules.conv.___torch_mangle_59.Conv2d
  bn1 : __torch__.torch.nn.modules.batchnorm.___torch_mangle_60.BatchNorm2d
  relu : __torch__.torch.nn.modules.activation.___torch_mangle_61.ReLU
  maxpool : __torch__.torch.nn.modules.pooling.___torch_mangle_62.MaxPool2d
  layer1 : __torch__.torch.nn.modules.container.___torch_mangle_75.Sequential
  layer2 : __torch__.torch.nn.modules.container.___torch_mangle_91.Sequential
  layer3 : __torch__.torch.nn.modules.container.___torch_mangle_107.Sequential
  layer4 : __torch__.torch.nn.modules.container.___torch_mangle_123.Sequential
  avgpool : __torch__.torch.nn.modules.pooling.___torch_mangle_124.AdaptiveAvgPool2d
  fc : __torch__.torch.nn.modules.linear.___torch_mangle_125.Linear
  def forward(self: __torch__.torchvision.models.resnet.___torch_mangle_126.ResNet,
    x: Tensor) -> Tensor:
    fc = self.fc
    avgpool = self.avgpool
    layer4 = self.layer4
    layer3 = self.layer3
    layer2 = self.layer2
    layer1 = self.layer1
    maxpool = self.maxpool
    relu = self.relu
    bn1 = self.bn1
    conv1 = self.conv1
    _0 = (bn1).forward((conv1).forward(x, ), )
    _1 = (maxpool).forward((relu).forward(_0, ), )
    _2 = (layer2).forward((layer1).forward(_1, ), )
    _3 = (layer4).forward((layer3).forward(_2, ), )
    input = torch.flatten((avgpool).forward(_3, ), 1)
    return (fc).forward(input, )
