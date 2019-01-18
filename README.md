# 1-wire-uart
单线通讯--串口

# 功能

> 借用串口协议进行单线通讯，目前仅适用于单主从结构，查询方式，异步传输

# 使用说明

> 编译为 ko 文件，加载文件后会生成 /dev/1-wire-uart 文件，通过操作 read write 进行通讯

# 例子程序

```
int main()
{
    unsigned char buf=0;
    int len=0;
    fd=open("/dev/1-wire-uart","wr");
    write(fd,&buf,1);
    print("put : 0x%x\n",buf);
    sleep(1);
    len=read(fd,&buf,1);
    print("got : 0x%x\n",buf);
}
```