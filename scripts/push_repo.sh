commit_str=$1

func()
{
    git add .
    git commit -m $1
    git push
    git log --oneline -3
}
func commit_str
