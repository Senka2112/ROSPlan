# A message used to publish a status notification from the Knowledge Base
# This status notifies that the KB has been updated.


uint8 ADD_KNOWLEDGE = 0
uint8 ADD_GOAL = 1
uint8 REMOVE_KNOWLEDGE = 2
uint8 REMOVE_GOAL = 3
uint8 ADD_METRIC = 4
uint8 REMOVE_METRIC = 5

std_msgs/Header header
string last_update_client
time last_update_time

uint8 update_type
rosplan_knowledge_msgs/KnowledgeItem knowledge
