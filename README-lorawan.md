# ns-3-lorawan

ns-3（dev 主干）+ LoRaWAN 模块的专用 fork，服务
[ms-lora-network-planner](../ms-lora-network-planner) 的 PDR 验证功能。

## 与上游的关系

- 基座：ns-3 dev 主干
- `src/lorawan`：以 **subtree** 合入
  [signetlabdei/lorawan](https://github.com/signetlabdei/lorawan) **v0.3.7**
  （完整提交历史保留；后续上游发版可直接
  `git subtree pull --prefix=src/lorawan <url> v0.x.x --squash` 合并）
- 相对上游 v0.3.7 的修改（升级时需重放）：
  - **ns-3-dev 适配**：IWYU 直接 include（59 文件）
  - `LogicalLoraChannelHelper::Resize()` — 显式信道计划
  - `GatewayLoraPhy::ClearFrequencies()` — 逐网关监听集
  - `SimpleGatewayLoraPhy::StartReceive` 频率过滤 — 上游声明了监听列表但从未执行
  - `AdrComponent` 新增 `MarginOffset` 属性 — 上游 ADR 把 SNR 余量吃满、链路贴
    灵敏度悬崖震荡；Semtech 规范应预留保护余量
- `scratch/lorawan-planner-sim.cc` — 规划项目的 PDR 仿真程序（信道计划/ADR/
  阴影衰落/网关去重/丢包原因计数，输入输出格式见源文件头注释）

## 构建

```bash
./ns3 configure --enable-examples --enable-tests -d optimized -- -DNS3_FAST_LINKERS=OFF
make -C cmake-cache scratch_lorawan-planner-sim -j8
# 产物: build/scratch/ns3-dev-lorawan-planner-sim-optimized
```

`NS3_FAST_LINKERS=OFF` 规避 homebrew lld 与新版 CommandLineTools SDK 的
TAPI 不兼容（macOS）。规划项目自动发现该产物，也可用 `MSLORA_NS3_BIN` 指定。
