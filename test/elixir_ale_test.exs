defmodule ElixirAleTest do
  use ExUnit.Case

  test "try to sense in pin7" do
{:ok, pid} = DHT11.start_link 7
DHT11.sense pid

    assert 1 + 1 == 2
  end
end
