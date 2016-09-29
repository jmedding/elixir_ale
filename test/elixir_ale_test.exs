defmodule ElixirAleTest do
  use ExUnit.Case

  test "try to sense in GPIO 4" do
    #GPIO 4 is pin 7 on the digrams
    {:ok, pid} = DHT11.start_link 4
    DHT11.sense pid

    assert 1 + 1 == 2
  end
end
