classdef taskManager < handle
    properties (SetAccess = private)
        taskType = 1;
        taskStatus = 0; % 1 on, 0 done
    end
    
    methods
        function set_task_type(self, tasktype)
            if tasktype == 1
                self.taskType = 1;
            elseif tasktype == -1
                self.taskType = -1;
            else
                error('wrong task type')
            end
        end
        
        function update_task_status(self)
        end
    end
end