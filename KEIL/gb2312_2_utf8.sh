echo "please wait for seconds"
find .  -name "*.c" -exec bash -c 'mkdir -p temp/$(dirname {}); iconv -f gb2312 -t utf-8 {} > temp/{} && mv temp/{} {}' \; && rm -rf temp
find .  -name "*.h" -exec bash -c 'mkdir -p temp/$(dirname {}); iconv -f gb2312 -t utf-8 {} > temp/{} && mv temp/{} {}' \; && rm -rf temp
echo "ok,enjoy!!!"
sleep 600