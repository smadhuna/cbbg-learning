def get_goal_string(object_dict, obj_list, obj_loc_list, goal_list, 
    goal_loc_list,env):
    """
        Returns
        ========
            str:
                A generic goal condition that will place every object based on
                its type and size at the correct goal.
    """

    # Append your goal condition to this string.
    #
    # Below are a few hints to help you along the way:
    # =================================================
    #
    # You can print the parameters to help you in coming up with a generalized
    # goal condition.
    #
    # You can also look at the (:objects) and (:init) of problem.pddl to give
    # you an idea of where your goal string is going wrong.
    #
    # Keep in mind the subject type and sizes of the bins and the books must
    # match.
    #
    # High level plans do not need actual co-ordinates, rather they use the
    # high-level locations found in the parameters of this method.
    #
    # Finally, there exists a way to write the goal condition using universal
    # and existential quantifiers.
    #
    # Executing wrong plans on Gazebo might not give you the right execution!
    #
    # Remember that the two environments have unique objects, goals and predicates!
    # 
    goal_string = "(:goal "

    if env == "bookWorld":
        #TODO: 
        # Replace this with the correct goal for bookWorld environment
        goal_string += "(forall (?b - book) (exists (?l - location ?t - bin ?s - subject ?sz - size) (and (Book_Subject ?b ?s) (Bin_Subject ?t ?s)(Book_At ?b ?l)(Bin_At ?t ?l)(Book_Size ?b ?sz)(Bin_Size ?t ?sz)))))"

    elif env == "cafeWorld":
        #TODO:
        # Replace this with the correct goal for cafeWorld environment
        goal_string += "(forall (?b - food) (exists (?l - location ?t - table ?s - food_type ?sz - size) (and (Food_Type ?b ?s) (Ordered ?t ?s)(Food_At ?b ?l)(Table_At ?t ?l)(Portion_Size ?b ?sz)(Ordered_Portion ?t ?sz)))))"
    
    return goal_string

def get_extra_credit_goal_string(object_dict, obj_list, obj_loc_list, 
    goal_list, goal_loc_list,env):
    """
        Returns
        ========
            str:
                A generic goal condition using universal and existential 
                quantifiers that will place every object based on its type and 
                size at the correct location
    """
    goal_string = "(:goal "
    
    if env == "bookWorld":
        goal_string += "(forall (?b - book) (exists (?l - location ?t - bin ?s - subject ?sz - size) (and (Book_Subject ?b ?s) (Bin_Subject ?t ?s)(Book_At ?b ?l)(Bin_At ?t ?l)(Book_Size ?b ?sz)(Bin_Size ?t ?sz)))))"
    elif env == "cafeWorld":
        goal_string += "(forall (?b - food) (exists (?l - location ?t - table ?s - food_type ?sz - size) (and (Food_Type ?b ?s) (Ordered ?t ?s)(Food_At ?b ?l)(Table_At ?t ?l)(Portion_Size ?b ?sz)(Ordered_Portion ?t ?sz)))))"


    # Implement a goal condition using forall/existential quantifiers for extra
    # credit!
    #
    # If this is the way you've already written the formula, then simply copy
    # over the contents from get_goal_string()!
    return goal_string


def sample_goal_condition(object_dict, obj_list, obj_loc_list, goal_list, 
    goal_loc_list):
    """
        Returns
        ========
            str:
                A generic goal condition that moves the robot to any one of
                the object locations.
    """

    # raise Exception("This code will not be considered for grading.")

    # You can replace the contents of get_goal_string() with the text below
    # to get an idea of what is expected.
    #
    # The goal condition in the stock task here is VASTLY different from the
    # expectation from you. Please review the homework documentation to identify
    # your task.
    #
    # Here are some instructions to run this in Gazebo.
    # 1. Replace the content of get_goal_string() with this method.
    # 2. rosrun hw2 refinement.py \
    #       --objtypes <object types> \
    #       --objcount <number of objects> \
    #       --seed <seed>
    # 3. rosrun hw2 gazebo.py

    # The generic goal condition here is to move the robot to a object location.
    #
    # The stock task below generates a generic goal condition that moves the
    # robot to a random object location and this is independent of the total 
    # number of locations and objects. 

    import random
    assert len(obj_loc_list) > 0
    i = random.randint(0, len(obj_loc_list) - 1)
    
    goal_string = "(:goal (and "
    goal_string += "(Robot_At tbot3 %s)" % (obj_loc_list[i])
    goal_string += "))\n"
    
    return goal_string