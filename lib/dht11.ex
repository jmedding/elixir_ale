defmodule DHT11 do
  use GenServer

  @moduledoc """
  This is an Elixir interface to the DHT11 via Linux GPIOs. Each DHT11 is an
  independent GenServer.
  """

  defmodule State do
    @moduledoc false
    defstruct port: nil,
              pin: 0,
              direction: nil,
              callbacks: []
  end

  @type pin_direction :: :input | :output
  @type int_direction :: :rising | :falling | :both

  # Public API
  @doc """
  Start and link a new GPIO GenServer. `pin` should be a valid
  GPIO pin number on the system and `pin_direction` should be
  `:input` or `:output`.
  """
  @spec start_link(integer, [term]) :: {:ok, pid}
  def start_link(pin, opts \\ []) do
    GenServer.start_link(__MODULE__, [pin], opts)
  end

  @doc """
  Free the resources associated with pin and stop the GenServer.
  """
  @spec release(pid) :: :ok
  def release(pid) do
    GenServer.cast pid, :release
  end

@doc """
  Sense the current temp and humidity value of the DHT11.
  """
  @spec sense(pid) :: :ok | {:error, term}
  def sense(pid) do
    GenServer.call pid, :sense
  end

  # gen_server callbacks
  def init([pin]) do
    executable = :code.priv_dir(:elixir_ale) ++ '/ale'
    port = Port.open({:spawn_executable, executable},
    [{:args, ["dht11", "#{pin}"]},
      {:packet, 2},
      :use_stdio,
      :binary,
      :exit_status])
    state = %State{port: port, pin: pin, direction: :input}
    {:ok, state}
  end

  def handle_call(:sense, _from, state) do
    {:ok, response} = call_port(state, :sense, [])
    {:reply, response, state}
  end
  
  def handle_cast(:release, state) do
    {:stop, :normal, state}
  end

  def handle_info({_, {:data, <<?n, message::binary>>}}, state) do
    msg = :erlang.binary_to_term(message)
    handle_port(msg, state)
  end

  defp call_port(state, command, arguments) do
    msg = {command, arguments}
    send state.port, {self, {:command, :erlang.term_to_binary(msg)}}
    receive do
      {_, {:data, <<?r,response::binary>>}} ->
        {:ok, :erlang.binary_to_term(response)}
    after
      1_000 -> :timedout
    end
  end

  defp handle_port({:gpio_interrupt, condition}, state) do
    #IO.puts "Got interrupt on pin #{state.pin}, #{condition}"
    msg = {:gpio_interrupt, state.pin, condition}
    for pid <- state.callbacks do
      send pid, msg
    end
    {:noreply, state}
  end

end
