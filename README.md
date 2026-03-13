# 

## (只执行一次)

git clone https://github.com/Fauna0916/TDPS.git
cd TDPS

git checkout dev

git config user.name "你的名字"
git config user.email "你的邮箱@xxx.com"

## 2. 日常开发流程

git checkout dev
git pull origin dev

git checkout -b feature/功能名

### 修改代码后

git add .
git commit -m "注释"

# 将你的分支推送到远程

git push -u origin feature/功能名


**第三步：发起 PR (合并请求)**

1. 打开 GitHub 仓库页面。
2. 页面会弹出黄色提示，点击 **"Compare & pull request"**。
3. 确保 **Base 为 dev**，**Compare 为你的分支**。
4. 填写说明，并**指定一位组员作为 Reviewer** 进行审核。
5. 等待组员审核通过并点击 **"Merge"**，你的代码就正式合入了！

