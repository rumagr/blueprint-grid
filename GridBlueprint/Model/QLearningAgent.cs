using System;
using System.Collections.Generic;
using Mars.Components.Services.Explorations;
using Mars.Interfaces.Agents;
using Mars.Interfaces.Annotations;
using Mars.Interfaces.Environments;
using Mars.Interfaces.Layers;

namespace GridBlueprint.Model;

//TODO implement Q-learning 

public class QLearningAgent : IAgent<GridLayer>, IPositionable
{
    #region Init

    /// <summary>
    ///     The initialization method of the SimpleAgent is executed once at the beginning of a simulation.
    ///     It sets an initial Position and an initial State and generates a list of movement directions.
    /// </summary>
    /// <param name="layer">The GridLayer that manages the agents</param>
    public void Init(GridLayer layer)
    {
        _layer = layer;
        Position = new Position(StartX, StartY);
        _directions = CreateMovementDirectionsList();
        _layer.QLearningAgentEnvironment.Insert(this);
        _qTable = new double[_layer.Width, _layer.Height][];
        InitQTable();
    }

    public void InitQTable()
    {
            // Initialize Q-table with zeros
        for (int i = 0; i < _layer.Width; i++)
        {
            for (int j = 0; j < _layer.Height; j++)
            {
                _qTable[i, j] = new double[4]; // Assuming 4 actions (up, down, left, right)
            }
        }
    }    


    #endregion

    #region Tick

    /// <summary>
    ///     The tick method of the SimpleAgent is executed during each time step of the simulation.
    ///     A SimpleAgent can move randomly. It must stay within the bounds of the GridLayer and cannot move onto grid
    ///     cells that are not routable.
    ///     Near the end of the simulation, a SimpleAgent removes itself from the simulation.
    /// </summary>
    public void Tick()
    {
        determineNextPosition();
    }

    #endregion

    #region Methods
    
    /// <summary>
    ///     Generates a list of eight movement directions that the agent uses for random movement.
    /// </summary>
    /// <returns>The list of movement directions</returns>
    private static List<Position> CreateMovementDirectionsList()
    {
        return new List<Position>
        {
            MovementDirections.North,
            MovementDirections.East,
            MovementDirections.South,
            MovementDirections.West,
        };
    }
    
    /// <summary>
    ///     Removes this agent from the simulation and, by extension, from the visualization.
    /// </summary>
    private void RemoveFromSimulation()
    {
        Console.WriteLine($"QLearningAgent {ID} is removing itself from the simulation.");
        _layer.QLearningAgentEnvironment.Remove(this);
        UnregisterAgentHandle.Invoke(_layer, this);
    }

    /// <summary>
    ///     Performs one random move, if possible, using the movement directions list.
    /// </summary>
    private void MoveRandomly()
    {
        var nextAction = _random.Next(_directions.Count);
        var nextDirection = _directions[nextAction];
        var newX = Position.X + nextDirection.X;
        var newY = Position.Y + nextDirection.Y;

        // Check if chosen move is within the bounds of the grid
        if (0 <= newX && newX < _layer.Width && 0 <= newY && newY < _layer.Height)
        {
            // Check if chosen move goes to a cell that is routable
            if (_layer.IsRoutable(newX, newY))
            {
                updateQTable(Position,nextAction , 0, new Position(newX, newY));
                Position = new Position(newX, newY);
                _layer.QLearningAgentEnvironment.MoveTo(this, new Position(newX, newY));
                Console.WriteLine("{0} moved to a new cell: {1}", GetType().Name, Position);
            }
            else if (_layer.IsExit((int) newX,(int) newY))
            {
                updateQTable(Position,nextAction , 10, new Position(newX, newY));
                Position = new Position(newX, newY);
                _layer.QLearningAgentEnvironment.MoveTo(this, new Position(newX, newY));
                Console.WriteLine("{0} moved to the exit cell: {1}", GetType().Name, Position);
                RemoveFromSimulation();
            }
            else
            { 
                updateQTable(Position,nextAction , -1, new Position(newX, newY));
                Console.WriteLine("{0} tried to move to a blocked cell: ({1}, {2})", GetType().Name, newX, newY);
                resetToStart();
            }
        }
        else
        {
            updateQTable(Position,nextAction , -1, new Position(newX, newY));
            Console.WriteLine("{0} tried to leave the world: ({1}, {2})", GetType().Name, newX, newY);
            resetToStart();
        }
    }
    
    /// <summary>
    /// resets the agent's position to the start position
    /// </summary>
    public void resetToStart()
    {
        Position = new Position(StartX, StartY);
        _layer.QLearningAgentEnvironment.MoveTo(this, new Position(StartX, StartY));
        Console.WriteLine("{0} was reset to the start position: {1}", GetType().Name, Position);
    }
    
    public void updateQTable(Position state, int action, double reward, Position nextState)
    {
        var currentStateX = (int) state.X;
        var currentStateY = (int) state.Y;
        var nextStateX = (int) nextState.X;
        var nextStateY = (int) nextState.Y;
        
        var oldValue = _qTable[currentStateX, currentStateY][action];
        var nextMax = GetBestActionForState(nextStateX, nextStateY);
        double newValue = 0;
        if(nextMax >= 0)
        {
            newValue = oldValue + alpha * (reward + gamma * _qTable[nextStateX,nextStateY][nextMax] - oldValue);
        }
        _qTable[currentStateX, currentStateY][action] = newValue;
        //TODO update Q-table using Q-learning formula
    }
    
    public void determineNextPosition()
    {
        if (IsTrueWithChance(epsilon))
        {
            MoveRandomly();
        }
        else 
        {
            int nextAction = GetBestActionForState((int) Position.X, (int) Position.Y);
            
            var nextDirection = _directions[nextAction];
            var newX = Position.X + nextDirection.X;
            var newY = Position.Y + nextDirection.Y;
            
            // Check if chosen move is within the bounds of the grid
            if (0 <= newX && newX < _layer.Width && 0 <= newY && newY < _layer.Height)
            {
                // Check if chosen move goes to a cell that is routable
                if (_layer.IsRoutable(newX, newY))
                {
                    updateQTable(Position, nextAction, 0, new Position(newX, newY));
                    Position = new Position(newX, newY);
                    _layer.QLearningAgentEnvironment.MoveTo(this, new Position(newX, newY));
                    Console.WriteLine("{0} moved to a new cell: {1}", GetType().Name, Position);
                }
                else if (_layer.IsExit((int) newX,(int) newY))
                {
                    updateQTable(Position, nextAction, 10, new Position(newX, newY));
                    Position = new Position(newX, newY);
                    _layer.QLearningAgentEnvironment.MoveTo(this, new Position(newX, newY));
                    Console.WriteLine("{0} moved to the exit cell: {1}", GetType().Name, Position);
                    RemoveFromSimulation();
                }
                else
                {
                    updateQTable(Position, nextAction, -1, new Position(newX, newY));
                    Console.WriteLine("{0} tried to move to a blocked cell: ({1}, {2})", GetType().Name, newX, newY);
                    resetToStart();
                }
            }
            else
            {
                updateQTable(Position, nextAction, -1, new Position(newX, newY));
                Console.WriteLine("{0} tried to leave the world: ({1}, {2})", GetType().Name, newX, newY);
                resetToStart();
            }
        }
    }
    
    public int GetBestActionForState(int x, int y)
    {
        if (x < 0 || x >= _layer.Width || y < 0 || y >= _layer.Height)
        {
            return -1; // Invalid state
        }
        double[] actions = _qTable[x, y];
        int maxActionIndex = 0;
        double maxValue = actions[0];

        for (int i = 1; i < actions.Length; i++)
        {
            if (actions[i] > maxValue)
            {
                maxValue = actions[i];
                maxActionIndex = i;
            }
            else if (actions[i] == maxValue)
            {
                if (IsTrueWithChance(0.5))
                {
                    maxValue = actions[i];
                    maxActionIndex = i; 
                }
            }
        }
        
        return maxActionIndex;
    }

    /// <summary>
    ///     Increments the agent's MeetingCounter property value.
    /// </summary>
    public void IncrementCounter()
    {
        MeetingCounter += 1;
    }
    
    private bool IsTrueWithChance(double chance)
    {
        Random random = new Random();
        return random.NextDouble() < chance;
    }
    
    
    #endregion

    #region Fields and Properties

    public Guid ID { get; set; }
    
    public Position Position { get; set; }
    
    [PropertyDescription(Name = "StartX")]
    public int StartX { get; set; }
    
    [PropertyDescription(Name = "StartY")]
    public int StartY { get; set; }

    public int MeetingCounter { get; private set; }

    public UnregisterAgent UnregisterAgentHandle { get; set; }
    
    public static double epsilon = 0.2; // Exploration rate
    public static double alpha = 0.8; // Learning rate
    public static double gamma = 0.6; // Discount factor
    
    private double[,][] _qTable;
    private GridLayer _layer;
    private List<Position> _directions;
    private readonly Random _random = new();

    #endregion
}