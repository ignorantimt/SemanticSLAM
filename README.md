

# SemanticSLAM

g2o依赖eigen 3.1.0
Semantic SLAM依赖eigen 3.4.0
注意修改cmake文件

build步骤：
- 使用工程目录下的thirdparty文件夹下的remake-deps.sh编译依赖项
- 使用src/SemanticSLAM目录下的eigenBuild.sh下载并编译两个版本的eigen
- 使用src/SemanticSLAM目录下的ThirdpartyBuild.sh下载并编译依赖项

