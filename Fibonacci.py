# Fibonacci Series
num = int(input("Enter upto Length: "))

fib1 = 0
fib2 = 1

print("Fibonacci Series:")
for i in range(num):
    print(fib1)
    fib1, fib2 = fib2, fib1 + fib2