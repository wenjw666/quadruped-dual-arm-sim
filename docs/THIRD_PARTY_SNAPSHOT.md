# 第三方源码快照

初始版本将工作区中的第三方代码作为普通源码目录保存，包括本地适配修改，不使用依赖未推送提交的 Git 子模块。原工作区的嵌套 `.git` 元数据保留在本机，不上传到本仓库。

`baseline/vendor_revisions.json` 记录可识别的上游地址、基准提交和本地修改文件清单。机器人描述包及其他无 Git 元数据的资源按当前目录快照保存；Robotiq 来源另见 `src/qda_description/meshes/robotiq_2f85/SOURCE.json`。

保留各组件已有 LICENSE、README、源码版权和 package.xml 声明。未确认来源的资产不作重新授权声明；此私有仓库用于保存当前项目基础。

`baseline/container-packages.tsv` 与 `baseline/python-packages.json` 为创建奠基提交时现有容器的软件版本清单，不是已验证的新机器安装锁文件。
