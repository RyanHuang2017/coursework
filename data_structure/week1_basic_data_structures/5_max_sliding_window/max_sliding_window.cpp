#include <iostream>
#include <vector>
#include <cassert>

using std::cin;
using std::cout;
using std::vector;
using std::max;

class StackWithMax {
    vector<int> stack;
    int maxElem;

  public:

    void Push(int value) {
        if (stack.empty()){
            maxElem = value;
            stack.push_back(value);
            return;
        }
        if (value > maxElem)
        {
            stack.push_back(2*value - maxElem);
            maxElem = value;
        }
        else
        {
            stack.push_back(value);
        }
    }

    int Pop() {
        assert(stack.size());
        int top_ = stack.back();
        stack.pop_back();
        if (top_ > maxElem){
            int prevMax = maxElem;
            maxElem = 2 * maxElem - top_;
            return prevMax;
        }
        return top_;
    }

    int Max() const {
        assert(stack.size());
        return maxElem;
    }

    bool Empty(){
        return stack.empty();
    }

    int Size(){
        return stack.size();
    }
};

struct QueueWithMax
{
    StackWithMax s1, s2;
    int maxElem;

    void enQueue(int x){
        int temp;
        while (!s1.Empty()){
            s2.Push(s1.Pop());
        }

        s1.Push(x);

        while(!s2.Empty()){
            s1.Push(s2.Pop());
        }
    }

    int deQueue(){
        assert(!s1.Empty());
        return s1.Pop();
    }

    int size(){
        return s1.Size();
    }

    int maxQueue(){
        return s1.Max();
    }
};

void max_sliding_window_naive(QueueWithMax const &A, int n, int w)
{
    QueueWithMax temp = A;
    QueueWithMax window;
    size_t i = 0;
    while (i < n - w + 1)
    {
        while(i == 0){
            for (size_t j = 0; j < w; j++){
                window.enQueue(temp.deQueue());
            }
            cout << window.maxQueue() << " ";
            break;
        }
        
        if (temp.size()){
            window.deQueue();
            window.enQueue(temp.deQueue());
            cout << window.maxQueue() << " ";
        }
        i ++;
    }
    return;
}


int main() {
    int n = 0;
    cin >> n;

    QueueWithMax A;
    int input;
    for (size_t i = 0; i < n; ++i)
    {
        cin >> input;
        A.enQueue(input);
    }  
    int w = 0;
    cin >> w;

    max_sliding_window_naive(A, n, w);

    return 0;
}
