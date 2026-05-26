# GitHub仓库创建和推送指南

## 📋 目录

1. [准备工作
2. [创建GitHub仓库](#2-创建github仓库
3. [添加远程仓库](#3-添加远程仓库
4. [推送到GitHub](#4-推送到github
5. [后续操作](#5-后续操作
6. [常见问题](#6-常见问题

---

## 1. 准备工作

### 1.1 检查Git配置

```bash
# 检查当前分支
git branch

# 检查当前提交
git log --oneline -5

# 检查远程仓库（如果已存在）
git remote -v
```

### 1.2 Git用户配置

如果你还没有配置Git用户信息：

```bash
git config --global user.name "Your Name"
git config --global user.email "your.email@example.com"

# 验证配置
git config --list
```

---

## 2. 创建GitHub仓库

### 2.1 通过网页创建（推荐）

1. 访问 GitHub: https://github.com/new

2. 填写仓库信息：
   - **Repository name**: `agv-ros2-system（或你喜欢的名称）
   - **Description**: "A complete AGV system based on ROS2 Humble with Docker deployment
   - **Visibility**: Public/Private（根据需要选择）
   - **Initialize repository with**: 都不要勾选（我们已有本地已经有代码了）

3. 点击 **Create repository**

### 2.2 使用GitHub CLI创建（如果已安装）

```bash
# 安装GitHub CLI（可选）
# https://cli.github.com/

# 登录
gh auth login

# 创建仓库
gh repo create agv-ros2-system --public --source=. --remote=origin --push
```

---

## 3. 添加远程仓库

### 3.1 复制仓库URL

创建仓库创建成功后，GitHub会显示仓库URL，类似于：
- HTTPS: `https://github.com/你的用户名/agv-ros2-system.git`
- SSH: `git@github.com:你的用户名/agv-ros2-system.git`

### 3.2 添加远程仓库

```bash
# 检查是否已有origin
git remote -v

# 如果没有，添加远程仓库
# 使用HTTPS方式（推荐新手）
git remote add origin https://github.com/你的用户名/agv-ros2-system.git

# 或使用SSH方式（需要配置SSH密钥）
git remote add origin git@github.com:你的用户名/agv-ros2-system.git

# 验证
git remote -v
```

### 3.3 配置SSH密钥（可选但推荐）

如果你使用SSH方式：

```bash
# 1. 生成SSH密钥（如果还没有）
ssh-keygen -t ed25519 -C "your.email@example.com"

# 2. 启动ssh-agent
eval "$(ssh-agent -s)"
ssh-add ~/.ssh/id_ed25519

# 3. 复制公钥到GitHub设置
cat ~/.ssh/id_ed25519.pub
# 然后访问 GitHub -> Settings -> SSH and GPG keys -> New SSH key

# 4. 测试连接
ssh -T git@github.com
```

---

## 4. 推送到GitHub

### 4.1 推送到主分支

```bash
# 查看当前分支
git branch

# 重命名当前分支为main（如果需要）
# git branch -M main

# 首次推送（设置上游分支）
git push -u origin main

# 如果是第一次推送，可能需要认证
# 输入GitHub用户名和Personal Access Token
```

### 4.2 推送其他分支（如果有）

```bash
# 查看所有分支
git branch -a

# 推送所有分支
git push --all origin

# 推送标签
git push --tags origin
```

---

## 5. 后续操作

### 5.1 克隆仓库（在其他机器上）

```bash
# 使用HTTPS
git clone https://github.com/你的用户名/agv-ros2-system.git

# 或使用SSH
git clone git@github.com:你的用户名/agv-ros2-system.git

# 进入目录
cd agv-ros2-system
```

### 5.2 日常工作流程

```bash
# 拉取最新更改
git pull origin main

# 查看状态
git status

# 创建新分支（用于新功能）
git checkout -b feature/new-feature

# 添加和提交
git add .
git commit -m "Description of changes"

# 推送分支
git push -u origin feature/new-feature
```

### 5.3 设置仓库设置

创建仓库后，建议设置：
1. 分支保护规则：Settings -> Branches
2. 团队权限：Settings -> Collaborators
3. 部署密钥：Settings -> Deploy keys
4. 自动化工作流：Actions

---

## 6. 常见问题

### 6.1 认证问题

**问题**: `fatal: Authentication failed`

**解决方案**:
1. 使用 Personal Access Token
   - GitHub -> Settings -> Developer settings -> Personal access tokens -> Tokens (classic)
   - 生成新token，勾选 `repo` 权限
   - 密码处粘贴token

```bash
# 使用Credential helper
git config --global credential.helper store

# 下次输入用户名和token
git push -u origin main
```

### 6.2 SSH权限问题

**问题**: `Permission denied (publickey)`

**解决方案**:
1. 确认SSH密钥已添加到GitHub
2. 确认密钥权限正确
```bash
ls -la ~/.ssh/
chmod 700 ~/.ssh
chmod 600 ~/.ssh/id_ed25519
```

### 6.3 已存在远程仓库

**问题**: `remote origin already exists`

**解决方案**:
```bash
# 删除原有远程仓库
git remote remove origin

# 重新添加
git remote add origin https://github.com/你的用户名/agv-ros2-system.git
```

### 6.4 推送被拒绝

**问题**: `Updates were rejected`

**解决方案**:
```bash
# 先拉取
git pull origin main --rebase

# 解决冲突后
git add .
git rebase --continue

# 再次推送
git push -u origin main
```

---

## 附录：完整快速命令速查表

```bash
# 查看当前仓库信息
git status
git log --oneline
git remote -v

# 添加远程
git remote add origin <url>

# 推送
git push -u origin main

# 拉取
git pull origin main

# 分支操作
git branch
git checkout -b new-branch
git checkout main

# 提交
git add .
git commit -m "message"
```

---

**完成后访问你的仓库地址，应该能看到所有文件了！🎉
