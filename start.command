#!/bin/zsh
set -e

cd "${0:A:h}"

if ! command -v node >/dev/null 2>&1 || ! command -v npm >/dev/null 2>&1; then
  echo "未检测到 Node.js / Node.js was not found."
  echo "请先安装 Node.js 22： https://nodejs.org/"
  echo "Install Node.js 22 first: https://nodejs.org/"
  read "?按回车键退出 / Press Enter to exit..."
  exit 1
fi

if [[ ! -d node_modules ]]; then
  echo "首次运行，正在安装依赖 / First run: installing dependencies..."
  npm ci
fi

echo "正在启动矩阵坐标实验室 / Starting Matrix Coordinate Lab..."
npm start
