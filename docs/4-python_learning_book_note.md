# Python 学习手册笔记

## 赋值语句
- 赋值语句建立对象引用值
- 变量名在首次赋值时会被创建
- 变量名在引用前必须先赋值

## 数据结构
- 列表拷贝:`new_list = old_list[:]`

## 作用域
- 如果一个变量在 `def` 内赋值, 它被定位在这个函数之内
- 如果一个变量在一个嵌套的 `def` 中赋值, 对于嵌套的函数来说, 它是 `nonlocal` 的
- 如果一个变量在 `def` 之外赋值, 它就是 `global` 的
- 全局作用域的作用范围仅限于单个文件。真全局的代替方法是变量名由模块文件隔开, 并且精确地导入一个模块文件才能够使用这个文件中定义的变量名
- 如果需要给一个在函数内部却位于模块文件顶层的变量名赋值, 需要在函数内部通过 `global` 声明；
- 如果需要给位于一个嵌套的 `def` 中的变量名赋值, 需要在嵌套函数内通过 `nonlocal` 声明
- 函数内部任何类型的赋值都会把一个名称划定为本地的 (包括 `=` 、 `import`)

## 参数
- 不可变参数通过值传递
- 可变参数通过指针进行引用传递 (列表、字典)

## 列表解析
- 在序列中映射函数 `map` (比 `for` 快一倍, 支持多入参的函数对多序列映射, 返回迭代器对象)
    ```py
    counters = [1, 2, 3, 4]
    func = lambda x: x + 10

    updated1 = []
    for x in counters:
        updated1.append(func(x))

    updated2 = list(map(func, counters))
    ```

## 模块
- import 和 from 是赋值语句
    - import 将整个模块对象赋值给一个变量
    - from 将一个或多个变量名赋值给另一个模块中同名的对象
- 模块间的变量名赋值
    ```py
    # change local x only
    from module_1 import x
    x = 42

    # change x in other module
    import module_1
    module_1.x = 42
    ```
- import 和 from 的对等性：from 总是会把整个模块导入到内存中, 再创建变量名引用导入模块中的同名对象
- `.pth` 文件可以显示存放需要导入 `sys.path` 的路径
- `from *` 语句只会复制出所有开头没有单下划线的变量名, 或预先定义在 `__all__` 列表中的变量名
