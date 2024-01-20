

func()
{
    git add .
    git commit -m $1
    git push
    git log --oneline -3
}
func $1
