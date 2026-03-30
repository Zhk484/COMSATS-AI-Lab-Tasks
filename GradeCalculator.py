#Grade Calculator

marks=int (input("Enter Your Marks:"))

if marks >100 or marks <0:
    print("Invalid Marks!")
elif marks >90:
    print("Your Grade is A")
elif marks >80:
    print("Your Grade is B")
elif marks >70:
    print("Your Grade is C")
elif marks >60:
    print("Your Grade is D")
elif marks >50:
    print("Your Grade is E")
elif marks <50:
    print("Your are Fail!")