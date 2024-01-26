echo "Digite o commit"
read message
git add .
git pull origin master
git commit -m"${message}"
git push -u origin master
git push https://fabioabdon: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@github.com/fabioabdon/nara2_ws.git --all

