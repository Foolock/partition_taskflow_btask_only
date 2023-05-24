def read_numbers_from_file(filename, lines):
    numbers = []
    with open(filename, 'r') as file:
        for line_number, line in enumerate(file, start=1):
            if line_number in lines:
                parts = line.split(':')
                if len(parts) == 2:
                    number = parts[1].strip().rstrip('%')
                    try:
                        number = float(number)
                        numbers.append(number)
                    except ValueError:
                        print(f"Invalid number found on line {line_number}: {number}")
    results = 0
    for i in range(len(numbers)):
        results += numbers[i]
    return results / len(numbers)

# Usage example
ftask = [5, 21, 37, 53, 69, 85, 101, 117, 133, 149]  # Specify the line numbers you want to read here
btask = [8, 24, 40, 56, 72, 88, 104, 120, 136, 152]  # Specify the line numbers you want to read here


file_path1 = './ac97_ctrl/log_t1_p1.txt'
file_path2 = './ac97_ctrl/log_t2_p2.txt'
file_path3 = './ac97_ctrl/log_t3_p3.txt'
file_path4 = './ac97_ctrl/log_t4_p4.txt'
file_path5 = './ac97_ctrl/log_t5_p5.txt'
file_path6 = './ac97_ctrl/log_t6_p6.txt'
file_path7 = './ac97_ctrl/log_t7_p7.txt'
file_path8 = './ac97_ctrl/log_t8_p8.txt'
file_path9 = './ac97_ctrl/log_t9_p9.txt'
file_path10 = './ac97_ctrl/log_t10_p10.txt'
file_path11 = './ac97_ctrl/log_t11_p11.txt'
file_path12 = './ac97_ctrl/log_t12_p12.txt'
file_path13 = './ac97_ctrl/log_t13_p13.txt'
file_path14 = './ac97_ctrl/log_t14_p14.txt'
file_path15 = './ac97_ctrl/log_t15_p15.txt'
file_path16 = './ac97_ctrl/log_t16_p16.txt'
file_path17 = './ac97_ctrl/log_t17_p17.txt'
file_path18 = './ac97_ctrl/log_t18_p18.txt'
file_path19 = './ac97_ctrl/log_t19_p19.txt'
file_path20 = './ac97_ctrl/log_t20_p20.txt'

results1 = read_numbers_from_file(file_path1, ftask)
results2 = read_numbers_from_file(file_path2, ftask)
results3 = read_numbers_from_file(file_path3, ftask)
results4 = read_numbers_from_file(file_path4, ftask)
results5 = read_numbers_from_file(file_path5, ftask)
results6 = read_numbers_from_file(file_path6, ftask)
results7 = read_numbers_from_file(file_path7, ftask)
results8 = read_numbers_from_file(file_path8, ftask)
results9 = read_numbers_from_file(file_path9, ftask)
results10 = read_numbers_from_file(file_path10, ftask)
results11 = read_numbers_from_file(file_path11, ftask)
results12 = read_numbers_from_file(file_path12, ftask)
results13 = read_numbers_from_file(file_path13, ftask)
results14 = read_numbers_from_file(file_path14, ftask)
results15 = read_numbers_from_file(file_path15, ftask)
results16 = read_numbers_from_file(file_path16, ftask)
results17 = read_numbers_from_file(file_path17, ftask)
results18 = read_numbers_from_file(file_path18, ftask)
results19 = read_numbers_from_file(file_path19, ftask)
results20 = read_numbers_from_file(file_path20, ftask)

print("ftask: ")
print(results1, end = ",")
print(results2, end = ",")
print(results3, end = ",")
print(results4, end = ",")
print(results5, end = ",")
print(results6, end = ",")
print(results7, end = ",")
print(results8, end = ",")
print(results9, end = ",")
print(results10, end = ",")
print(results11, end = ",")
print(results12, end = ",")
print(results13, end = ",")
print(results14, end = ",")
print(results15, end = ",")
print(results16, end = ",")
print(results17, end = ",")
print(results18, end = ",")
print(results19, end = ",")
print(results20)


results1 = read_numbers_from_file(file_path1, btask)
results2 = read_numbers_from_file(file_path2, btask)
results3 = read_numbers_from_file(file_path3, btask)
results4 = read_numbers_from_file(file_path4, btask)
results5 = read_numbers_from_file(file_path5, btask)
results6 = read_numbers_from_file(file_path6, btask)
results7 = read_numbers_from_file(file_path7, btask)
results8 = read_numbers_from_file(file_path8, btask)
results9 = read_numbers_from_file(file_path9, btask)
results10 = read_numbers_from_file(file_path10, btask)
results11 = read_numbers_from_file(file_path11, btask)
results12 = read_numbers_from_file(file_path12, btask)
results13 = read_numbers_from_file(file_path13, btask)
results14 = read_numbers_from_file(file_path14, btask)
results15 = read_numbers_from_file(file_path15, btask)
results16 = read_numbers_from_file(file_path16, btask)
results17 = read_numbers_from_file(file_path17, btask)
results18 = read_numbers_from_file(file_path18, btask)
results19 = read_numbers_from_file(file_path19, btask)
results20 = read_numbers_from_file(file_path20, btask)

print("btask: ")
print(results1, end = ",")
print(results2, end = ",")
print(results3, end = ",")
print(results4, end = ",")
print(results5, end = ",")
print(results6, end = ",")
print(results7, end = ",")
print(results8, end = ",")
print(results9, end = ",")
print(results10, end = ",")
print(results11, end = ",")
print(results12, end = ",")
print(results13, end = ",")
print(results14, end = ",")
print(results15, end = ",")
print(results16, end = ",")
print(results17, end = ",")
print(results18, end = ",")
print(results19, end = ",")
print(results20)























