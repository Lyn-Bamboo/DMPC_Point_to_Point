classdef Agent
    properties
        flag_arrive = 0;
        pos = [];
        v = [0, 0, 0];
        a = [0, 0, 0];
        goal = [];
        near_agent_index = [];
        posN=[];
        vN = [];
        aN = [];
        epsilon = [];
        esp = 5e-2;
    end

    methods
        function obj = Agent(pos, goal)
            obj.pos = pos;
            obj.goal = goal;
        end

        function obj = update(obj)
            obj.pos = obj.posN(1,:);
            obj.v = obj.vN(1,:);
            obj.a = obj.aN(1,:);
        end

        function obj = checkarrive(obj)
            if abs(obj.goal - obj.pos) < obj.esp
                obj.flag_arrive = 1;
            end
        end

    end

end
