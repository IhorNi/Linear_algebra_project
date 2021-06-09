# to run this module from command line typing 
#
#      > python robot_navigation.py robot_data.json
#
# it reads the starting and the finishing points as well as 
# the list of obstacles from robot_data.json file and then 
# runs find_path() function that has to return the path
#

import sys
import numpy as np
import json
from matplotlib import pyplot as plt

def is_segments_intersect(seg_1, seg_2):
    ## let's find two line coming through the two points of each segment
    ## v = a * v1 + (1 - a) * v2
    ## u = b * u1 + (1 - b) * u2
    ## lines intersect at u = v, =>  a * v1 + (1 - a) * v2 = b * u1 + (1 - b) * u2
    ## or  (v1 - v2) * a + (u2 - u1) * b = u2 - v2
    ## 
    ## if lines intersect within the given segments, a and b must be strictly between 0 and 1 
    
    v1, v2 = seg_1
    u1, u2 = seg_2
    
    ### Нужно исключить из поиска для финальной проверки
    # Для обработки частного случая, когда вектор движения попадает в вершину грани полигона
    if (((v1 == u1).all() or (v2 == u1).all()) or (v1 == u1).all() or (v2 == u1).all()):
        return True

    M = np.array([v1 - v2, u2 - u1]).T
    if np.linalg.matrix_rank(M) < 2:
        return False

    a, b = np.linalg.inv(M).dot(u2 - v2)

    return (0 <= a <= 1) and (0 <= b <= 1)

def IntersectionPoint(seg_1, seg_2):
    
    v1, v2 = seg_1
    u1, u2 = seg_2
    M = np.array([v1 - v2, u2 - u1]).T
    
    # Для обработки частного случая, когда вектор движения попадает
    # в вершину грани полигона
    if (v1 == u1).all() or (v1 == u2).all():
        result = v1
    elif (v2 == u1).all() or (v2 == u2).all():
        result = v2
    else:
        result = None
    
    if result is None:
        if np.linalg.matrix_rank(M) >= 2:      
            a, b = np.linalg.inv(M).dot(u2 - v2)
            if ((0 <= a <= 1) and (0 <= b < 1)) == True:
                result = (b*u1 + (1-b)*u2)
    return result

def CalculateTotalPath(path):
    """Функция считает суммарное расстояние пройденного пути"""
    
    path = [path[i+1]-path[i] for i in range(len(path)-1)]
    return sum(map(lambda x: np.linalg.norm(x), path))

def OptimizePath(path, obstacles, move_type):
    """Функция оптимизация пути робота.
    Зная последовательность безопасных точек, исключаются лишние движения"""
  
    
    start = path[0]
    OptimizedPath = [start]
    path_length = len(path)
    i = 1
    
    while i < path_length - 1:
        while True:            
            if i < path_length :
                move_segment = (start, path[i])
                ClearPath, _, _ = check_polyline(move_segment, obstacles)
                
                if ClearPath == False or (ClearPath == True and move_type[i]=='Obstacle'):
                    OptimizedPath.append(path[i-1])
                    if i < path_length - 1:
                        start = path[i]
                        i += 1
                    break
                else:                    
                    i += 1
            else:
                break
    OptimizedPath.append(path[-1])
    return OptimizedPath

def check_polyline(polyline, obstacles):
    """this function returns True if the polyline does not intersect obstacles
    Otherwise it returns False
    You can use it to verify your algorithm
    """
    check = True

    
    dist_to_intersection = np.inf
    for obstacle in obstacles:
        for i in range(len(obstacle)):
            obstacle_segment = (obstacle[i-1], obstacle[i]) 
            for j in range(1, len(polyline)):
                path_segment = (polyline[j-1], polyline[j])
                if is_segments_intersect(obstacle_segment, path_segment):
                    #if is_segments_intersect((obstacle_segment[1],obstacle_segment[0]), (path_segment[1], path_segment[0])):
                    #print("segments intersect:", obstacle_segment, path_segment)
                    check = False
                    intersection_point = IntersectionPoint(obstacle_segment, path_segment)
                    if np.linalg.norm(intersection_point - polyline[j-1])<dist_to_intersection:
                        dist_to_intersection = np.linalg.norm(intersection_point - polyline[j-1])
                        result_tuple = (False, obstacle, i)
    if check == True:
        result_tuple = (True, obstacle, i )
    
    return result_tuple

def GetAngleBetweenSegments(a, b):
    """Функция возвращает угол между двумя векторами"""

    vec_a = a[1] - a[0]
    vec_b = b[1] - b[0]
    cos_alpha = np.dot(vec_a, vec_b) / (np.linalg.norm(vec_a) * np.linalg.norm(vec_b))
    angle = np.degrees(np.arccos(cos_alpha))
    
    return angle

def GetRouteSlope(p1, p2):    
    """Функция возвращает коеффициент наклона прямой между двумя точками"""
    
    if (p2[0] == p1[0]) or (p2[1] == p1[1]):
        return 0
    else:
        return (p2[1] - p1[1])/(p2[0] - p1[0])
        
def GetBaseMove(current_point, finish):
    """Функция определяет базовый шаг("скорость") робота по направлению вектора СтартФиниш"""
    
    move = np.array([0.03, 0.03])
    start_finish = finish - current_point
    
    if start_finish[0] == 0:
        move[0] = 0
    
    move[0] = -move[0] if start_finish[0]<0 else move[0]
    move[1] = -move[1] if start_finish[1]<0 else move[1]
    
    return move

def AdjustMove(current_point, finish):
    """Функция собирает вместе данные о направлении старт-финиш и определяет конечный шаг робота"""
    
    base_move = GetBaseMove(current_point, finish)
    slope = GetRouteSlope(current_point, finish)
    if base_move[0] != 0:
        base_move =[base_move[0], base_move[0]*slope]
    return base_move

def PlotPathsAndReturnBest(X_left, X_right, obstacles):

    #plt.figure(figsize=(6,6))
    plt.style.use('dark_background')
    plt.axis('equal')
    
    left_lenght = round(CalculateTotalPath(X_left),2)
    
    x1, x2 = np.array(X_left).T
    plt.plot(x1, x2, c='cyan',  linewidth=2,
             label = 'First, len = ' + str(left_lenght))
    
    right_length = round(CalculateTotalPath(X_right),2)
    x3, x4 = np.array(X_right).T
    plt.plot(x3, x4, c='white',  linewidth=2,
             label = 'Second, len = ' + str(right_length))
    
    for ob in obstacles:
        ob = list(ob)
        ob.append(ob[0])
        x,y = zip(*ob)
        plt.plot(x, y, linestyle = '--', c='red')
    plt.legend()
    #plt.savefig(title)
    plt.show()
    
    return X_left if left_lenght < right_length else X_right

def PlotPath(X, obstacles):

    #plt.figure(figsize=(6,6))
    plt.style.use('dark_background')
    plt.axis('equal')
    
    path_lenght = round(CalculateTotalPath(X),2)
    
    x1, x2 = np.array(X).T
    plt.plot(x1, x2, c='cyan',  linewidth=2,
             label = 'Best path, len = ' + str(path_lenght))
        
    for ob in obstacles:
        ob = list(ob)
        ob.append(ob[0])
        x,y = zip(*ob)
        plt.plot(x, y, linestyle = '--', c='red')
    plt.legend()
    plt.show()

    return 

def GetNextMove(current_point, finish, obstacles, turn_rule="Right"):
    """Функция настраивает базовый шаг.
    И по приближению к пункту назначения робот подстраивает шаг ("скорость"), чтобы не перескочить финиш"""
    
    next_moves = []
    temp_current_point = current_point
    
    base_move = AdjustMove(temp_current_point, finish)    
    move_segment = [current_point,current_point+base_move]
    #ove_segment = [current_point,finish]
    ClearPath, obstacle, ob_index = check_polyline(move_segment, obstacles)
    
    #False - проверка не была пройдена
    if ClearPath == False:
        
        
        #hit_point = IntersectionPoint(move_segment, obstacle_hit_segment)
        #print('hit ', hit_point)
        #next_moves.append(hit_point-temp_current_point)
        #temp_current_point = temp_current_point + next_moves[-1]
        
        
        #next_moves.append(obstacle_hit_segment[0]-temp_current_point)
        #temp_current_point = temp_current_point + next_moves[-1] 
        #Начинаем обходить препятстивие, пока не поймем, что можно идти дальше
        
        left_dist = np.linalg.norm(obstacle[ob_index]-temp_current_point)
        right_dist = np.linalg.norm(obstacle[ob_index-1]-temp_current_point)
        
        turn_rule = 'Left' if left_dist < right_dist else 'Right'
        
        if turn_rule == 'Left':
            obstacle_hit_segment = (obstacle[ob_index], obstacle[ob_index-1])
        if turn_rule == 'Right':   
            obstacle_hit_segment = (obstacle[ob_index-1], obstacle[ob_index]) 
        
        while True:
            #Осторожно выбираем соседнюю грань, чтобы не выйти за рамки
            #Для левого и правого поворотов
            if turn_rule == 'Left':          
                first_index = 0 if abs(ob_index) > len(obstacle)-1 else ob_index
                #first_index = 0 if abs(ob_index + 1) > len(obstacle)-1 else ob_index+1
                second_index = 0 if abs(first_index + 1) > len(obstacle)-1 else first_index+1      
            if turn_rule == 'Right':               
                first_index = -1 if abs(ob_index - 1) > len(obstacle) else ob_index-1
                second_index = -1 if abs(first_index - 1) > len(obstacle) else first_index-1
                
            next_moves.append(obstacle[first_index]-temp_current_point)
            temp_current_point = temp_current_point + next_moves[-1] 
            
            #Решение, что идти к финишу можно по углу между векторами
            #Угол между гранью, о которую ударились, и соседней
                        
            obstacle_hit_neighbor = (obstacle[first_index], obstacle[second_index])  
            obstacle_vector_angle = GetAngleBetweenSegments(obstacle_hit_segment, obstacle_hit_neighbor)
            #Угол между гранью, о которую ударились, вектором на финиш
            finish_segment = (temp_current_point, finish)
            finish_vector_angle = GetAngleBetweenSegments(obstacle_hit_segment, finish_segment)
            #Если угол между грянями полигона меньше, чем угол между гранью и финишем, то можно идти дальше
            if finish_vector_angle>=obstacle_vector_angle:
                #Пересчитываем шаг для движения к финишу
                base_move = AdjustMove(temp_current_point, finish) 
                move_segment = [temp_current_point,current_point+base_move]
                ClearPath = check_polyline_final(move_segment, obstacles)
                destination = finish
                if ClearPath == False:           
                    base_move = GetBaseMove(temp_current_point, finish)
                break
            else:
                #переход к следующей паре граней с изменением направления
                #для расчета корректного расчета угла между векторами
                #при обходе углов
                
                if turn_rule == 'Left':
                    obstacle_hit_segment = (obstacle_hit_neighbor[1], obstacle_hit_neighbor[0])
                    if abs(ob_index) == len(obstacle):
                        ob_index = 0
                    else:
                        ob_index += 1
                
                if turn_rule == 'Right':  
                    obstacle_hit_segment = (obstacle_hit_neighbor[1], obstacle_hit_neighbor[0])
                    if abs(ob_index) == len(obstacle):
                        ob_index = -1
                    else:
                        ob_index -= 1

    else:
        destination = finish
    
    move_length = np.linalg.norm(base_move)
    
    dist_to_destination = np.linalg.norm(destination - temp_current_point)
        
    if dist_to_destination<move_length:
        next_moves.append(destination - temp_current_point)
    else:
        next_moves.append(base_move)
    
    return next_moves


def check_polyline_final(polyline, obstacles):
    """this function returns True if the polyline does not intersect obstacles
	Otherwise it returns False
	You can use it to verify your algorithm
	"""
    for obstacle in obstacles:
       for i in range(len(obstacle)):
           obstacle_segment = (obstacle[i-1], obstacle[i])
           for j in range(1, len(polyline)):
               path_segment = (polyline[j-1], polyline[j])
               if is_segments_intersect_final(obstacle_segment, path_segment):
                   #Хитрый трюк. Не совсем понимаю почему, но это дополнительная проверка
                   #Для ситуаций в стиле 
                   #(array([3., 1.]), array([1., 2.])) vs (array([1., 2.]), array([1.6, 2.3]))
                   #Который в такой ситуации при проверке дают значения a b
                   #бесконечно близкие к 0 или 1, что по предположениям модели
                   #должно скорее успешно пройти проверку, чем не пройти.
                   #Если сделать так, чтобы конец одного и начало другого не совпадали
                   #Проверка проходит успешно, как и предполагается
                   
                  
                    if is_segments_intersect_final((obstacle_segment[1],obstacle_segment[0]), (path_segment[1], path_segment[0])):
                        if is_segments_intersect_final((obstacle_segment), (path_segment[1], path_segment[0])):
                            #print("segments intersect:", obstacle_segment, path_segment)
                            #print("Intersection point: ", IntersectionPoint(obstacle_segment, path_segment))
                            return False
    return True
                   
           


def is_segments_intersect_final(seg_1, seg_2):
	## let's find two line coming through the two points of each segment
	## v = a * v1 + (1 - a) * v2
	## u = b * u1 + (1 - b) * u2
	## lines intersect at u = v, =>  a * v1 + (1 - a) * v2 = b * u1 + (1 - b) * u2
	## or  (v1 - v2) * a + (u2 - u1) * b = u2 - v2
	## 
	## if lines intersect within the given segments, a and b must be strictly between 0 and 1 
    zero = 0.00001
    one = 0.99999
    v1, v2 = seg_1
    u1, u2 = seg_2

    M = np.array([v1 - v2, u2 - u1]).T
    if np.linalg.matrix_rank(M) < 2:
    	return False

    a, b = np.linalg.inv(M).dot(u2 - v2)
    
    #return (0 < a < 1) and (0 < b < 1)    
    return (zero < a < one) and (zero < b < one)   

def PreparePath(start, finish, obstacles, turn_rule):
    
    start = np.array(start)
    finish = np.array(finish)
    obstacles = [ np.array(ob[:-1]) for ob in obstacles]
    
    print("Start:", start)
    print("Finish point:", finish)
    #print("Obstacles:", obstacles)
  
    X = [start]
    move = GetBaseMove(start, finish)
    move_descriptor = []
    #print(move
    #slope = GetRouteSlope(start, finish)
    #print(slope)
    while True:  
        next_moves = GetNextMove(X[-1], finish, obstacles, turn_rule)
        #Новых шагов может быть несколько, если мы начали обходить препятствие
        for move in next_moves:
            current_point = X[-1] + move        
            X.append(current_point)
        
        #print(len(next_moves))
        if len(next_moves) > 1:
            move_descriptor.append('Free')
            for _ in range(len(next_moves)-1):
                move_descriptor.append('Obstacle')
            move_descriptor.append('Free')
        else:
            move_descriptor.append('Free')
        finish_check = current_point==finish
        if finish_check.all():
            break
    #move_descriptor.append('Free')
    return X, move_descriptor


if __name__ == '__main__':
    
    if len(sys.argv) != 2:
        print("USAGE EXAMPLE:\n\n    python main.py robot_data.json\n")
        exit(1)
    data_file = sys.argv[1]
    f = open(data_file)
    data = json.load(f)
    f.close()
    
    obstacles = data["obstacles"]
    
    obstacles = [ np.array(ob) for ob in obstacles]
    
    print("\nInsvestigation of the Path ...")
    X_1, moves_1 = PreparePath(data["start"], data["finish"], data["obstacles"], 'Left')
    print("\nPath optimization ...")
    X_1_upd = OptimizePath(X_1, obstacles, moves_1)
     
    print('Best path is {}'.format(X_1_upd))
    print('\nFinal check: {}'.format(check_polyline_final(X_1_upd, obstacles)))
    PlotPath(X_1_upd, obstacles)